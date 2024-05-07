"""
RRT_Star path planning algorithm
Adapted from: https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning
"""

from typing import List
import os
import time
import datetime
import numpy as np

import utils, plotting
from environment import Environment, Node
from utils import Coordinate, PortPair

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"


class RRT_Star:
    def __init__(self,
                 land_filepath,
                 boat_filepath,
                 weather_dirpath,
                 weather_type,
                 portpair:PortPair,
                 datecycle:datetime,
                 t_max,
                 step_len, # [nmi]
                 search_radius, # [nmi]
                 goal_sample_rate, 
                 iter_max,
                 verbose=False,
                 log=False):
        self.s_start = Node(portpair.p_start, elapsed=0.0)
        self.s_goal = Node(portpair.p_end)
        self.route_name = f'{portpair.shortname}.{weather_type}.{datecycle.strftime('%Y%m%d%H')}'
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.verbose = verbose
        self.log = log

        self.nodes:List[Node] = [self.s_start]
        self.route = None

        if self.verbose: print('Setting up environment', end='... ', flush=True)
        self.env = Environment(land_filepath, boat_filepath, weather_dirpath, weather_type, portpair.lon_range, portpair.lat_range, datecycle, tf=t_max)
        if self.verbose: print('Done!')

    def planning(self):
        timestart = time.time()
        if self.verbose: print(f'Beginning {self.route_name} RRT*')
        for k in range(self.iter_max):
            if self.verbose and (k+1)%500==0: print('  ' + f'iter {k+1}', flush=True)
            if self.log and (k+1)%500==0: 
                plotting.plot(self.env,
                              nodelist=self.nodes,
                              filepath=ROOT_DIR+f'route_optimizer/output/log/{self.route_name}_iter{k+1}.png')

            coord_rand = self.generate_random_coord()
            node_near = self.nearest_neighbor(coord_rand)
            coord_new = self.new_state(node_near.coord, coord_rand)

            if not self.env.is_collision(node_near.coord, coord_new):
                node_new = Node(coord_new, parent=node_near)
                node_new.elapsed = self.get_new_elapsed(node_near, node_new)
                self.nodes.append(node_new)

                neighbor_indices = self.find_near_neighbors(node_new.coord)
                if neighbor_indices:
                    self.choose_parent(node_new, neighbor_indices)
                    self.rewire(node_new, neighbor_indices)

        index = self.search_goal_parent()
        self.route = self.extract_path(self.nodes[index])

        duration = time.time() - timestart
        if self.verbose: print(f'Completed {self.route_name} RRT* ({datetime.timedelta(seconds=duration)})')

        np.savetxt(ROOT_DIR+f'route_optimizer/output/routes/{self.route_name}.csv', self.route, delimiter=',')
        with open(ROOT_DIR+f'route_optimizer/output/route_times.csv','a') as file: file.write(f'{self.route_name},{self.env.time_route(self.route)}\n')
        plotting.plot(self.env,
                      route=self.route, nodelist=self.nodes,
                      filepath=ROOT_DIR+f'route_optimizer/output/search_plots/{self.route_name}.png')

    def new_state(self, coord_start:Coordinate, coord_goal:Coordinate) -> Coordinate:
        dist = min(self.step_len, utils.haversine_distance(coord_start, coord_goal))
        theta = utils.angle(coord_start, coord_goal)
        coord_new = utils.transform_coord(coord_start, dist, theta)

        return coord_new

    def choose_parent(self, node_new:Node, neighbor_indices):
        costs = [self.get_new_elapsed(self.nodes[i], node_new) for i in neighbor_indices]
        cost_min_index = np.argmin(costs)

        node_new.parent = self.nodes[neighbor_indices[cost_min_index]]
        node_new.elapsed = costs[cost_min_index]

    def rewire(self, node_new:Node, neighbor_indices):
        for i in neighbor_indices:
            node_neighbor:Node = self.nodes[i]

            new_elapsed = self.get_new_elapsed(node_new, node_neighbor)
            if new_elapsed < node_neighbor.elapsed:
                node_neighbor.parent = node_new
                node_neighbor.elapsed = new_elapsed

    def search_goal_parent(self):
        dist_list = [utils.haversine_distance(nd.coord, self.s_goal.coord) for nd in self.nodes]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) == 0:
            return len(self.nodes) - 1

        cost_list = [self.get_new_elapsed(self.nodes[i], self.s_goal) for i in node_index
                     if not self.env.is_collision(self.nodes[i].coord, self.s_goal.coord)]
        return node_index[np.argmin(cost_list)]

    def get_new_elapsed(self, node_start:Node, node_end:Node):
        ''' Resulting elapsed time to get to node_end if it is connected to node_start '''
        return node_start.elapsed + self.env.time_to_go(node_start, node_end)

    def generate_random_coord(self) -> Coordinate:
        if np.random.random() < self.goal_sample_rate:
            return self.s_goal.coord

        return Coordinate(np.random.uniform(self.env.lon_range[0], self.env.lon_range[1]),
                          np.random.uniform(self.env.lat_range[0], self.env.lat_range[1]))

    def find_near_neighbors(self, coord_new:Coordinate):
        n = len(self.nodes) + 1
        r = min(self.search_radius * np.sqrt(np.log(n)/n), self.step_len)

        dist_table = [utils.haversine_distance(coord_new, nd.coord) for nd in self.nodes]
        dist_table_index = [i for i in range(len(dist_table)) if dist_table[i] <= r and
                            not self.env.is_collision(coord_new, self.nodes[i].coord)]

        return dist_table_index

    def nearest_neighbor(self, coord:Coordinate) -> Node:
        return self.nodes[np.argmin([utils.haversine_distance(coord, nd.coord) for nd in self.nodes])]

    def extract_path(self, node_end:Node):
        path = [self.s_goal.coord.point()]
        node = node_end

        while node.parent is not None:
            path.append(node.coord.point())
            node = node.parent
        path.append(node.coord.point())

        return np.array(path[::-1])