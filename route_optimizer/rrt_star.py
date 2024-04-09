"""
RRT_Star path planning algorithm
Adapted from: https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning
"""

import os
import numpy as np

import utils, environment, plotting
from utils import Node

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"


class RRT_Star:
    def __init__(self,
                 land_filepath,
                 weather_filepath,
                 boat_filepath,
                 lon_range,
                 lat_range,
                 x_start,
                 x_goal,
                 route_name,
                 step_len, # [nmi]
                 search_radius, # [nmi]
                 goal_sample_rate, 
                 iter_max,
                 verbose=False,
                 log=False):
        self.s_start = Node(x_start, cost=0.0)
        self.s_goal = Node(x_goal)
        self.route_name = route_name
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.verbose = verbose
        self.log = log

        self.nodes = [self.s_start]
        self.route = None

        if self.verbose: print('Setting up environment', end='... ')
        self.env = environment.Env(land_filepath, weather_filepath, boat_filepath, lon_range, lat_range)
        if self.verbose: print('Done!')

    def planning(self):
        if self.verbose: print(f'Beginning {self.route_name} RRT*')
        for k in range(self.iter_max):
            if self.verbose and (k+1)%500==0: print('  ' + f'iter {k+1}', flush=True)
            if self.log and (k+1)%500==0: 
                plotting.plot(self.env, f'{self.route_name}, rrt* iter = {k+1}',
                              nodelist=self.nodes,
                              filepath=ROOT_DIR+f'route_optimizer/output/log/{self.route_name}_iter{k+1}.png')

            node_rand = self.generate_random_node()
            node_near = self.nearest_neighbor(node_rand)
            node_new = self.new_state(node_near, node_rand)

            if not self.env.is_collision(node_near, node_new):
                node_new.parent = node_near
                node_new.cost = self.get_new_cost(node_near, node_new)
                self.nodes.append(node_new)

                neighbor_indices = self.find_near_neighbors(node_new)
                if neighbor_indices:
                    self.choose_parent(node_new, neighbor_indices)
                    self.rewire(node_new, neighbor_indices)

        index = self.search_goal_parent()
        self.route = self.extract_path(self.nodes[index])
        if self.verbose: print(f'Completed {self.route_name} RRT*')

        np.savetxt(ROOT_DIR+f'route_optimizer/output/routes/{self.route_name}.csv', self.route, delimiter=',')
        with open(ROOT_DIR+f'route_optimizer/output/route_times.csv','a') as file: file.write(f'{self.route_name},{self.env.time_route(self.route)}\n')
        plotting.plot(self.env, f'{self.route_name} rrt*',
                      route=self.route, nodelist=self.nodes,
                      filepath=ROOT_DIR+f'route_optimizer/output/search_plots/{self.route_name}.png')

    def new_state(self, node_start:Node, node_goal:Node):
        dist = min(self.step_len, utils.haversine_distance(node_start, node_goal))
        theta = utils.angle(node_start, node_goal)

        node_new = utils.transform_node(node_start, dist, theta)

        return node_new

    def choose_parent(self, node_new:Node, neighbor_indices):
        costs = [self.get_new_cost(self.nodes[i], node_new) for i in neighbor_indices]
        cost_min_index = np.argmin(costs)

        node_new.parent = self.nodes[neighbor_indices[cost_min_index]]
        node_new.cost = costs[cost_min_index]

    def rewire(self, node_new, neighbor_indices):
        for i in neighbor_indices:
            node_neighbor = self.nodes[i]

            new_cost = self.get_new_cost(node_new, node_neighbor)
            if node_neighbor.cost > new_cost:
                node_neighbor.parent = node_new
                node_neighbor.cost = new_cost

    def search_goal_parent(self):
        dist_list = [utils.haversine_distance(nd, self.s_goal) for nd in self.nodes]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) == 0:
            return len(self.nodes) - 1

        cost_list = [self.get_new_cost(self.nodes[i], self.s_goal) for i in node_index
                     if not self.env.is_collision(self.nodes[i], self.s_goal)]
        return node_index[np.argmin(cost_list)]

    def get_new_cost(self, node_start, node_end):
        ''' Resulting cost for node_end if it is connected to node_start '''
        time_to_go = self.env.time_to_go(node_start, node_end)
        return node_start.cost + time_to_go

    def generate_random_node(self):
        if np.random.random() < self.goal_sample_rate:
            return self.s_goal

        return Node((np.random.uniform(self.env.lon_range[0], self.env.lon_range[1]),
                     np.random.uniform(self.env.lat_range[0], self.env.lat_range[1])))

    def find_near_neighbors(self, node_new):
        n = len(self.nodes) + 1
        r = min(self.search_radius * np.sqrt(np.log(n)/n), self.step_len)

        dist_table = [utils.haversine_distance(node_new, nd) for nd in self.nodes]
        dist_table_index = [i for i in range(len(dist_table)) if dist_table[i] <= r and
                            not self.env.is_collision(node_new, self.nodes[i])]

        return dist_table_index

    def nearest_neighbor(self, n):
        return self.nodes[np.argmin([utils.haversine_distance(n, nd) for nd in self.nodes])]

    def extract_path(self, node_end:Node):
        path = [self.s_goal.point()]
        node = node_end

        while node.parent is not None:
            path.append(node.point())
            node = node.parent
        path.append(node.point())

        return np.array(path[::-1])