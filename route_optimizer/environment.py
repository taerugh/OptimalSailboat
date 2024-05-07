"""
Environment object for route_optimizer
author: @taerugh
"""

import os
import numpy as np
from typing import Optional, Self
from datetime import datetime
import shapely.geometry as geometry

import utils
from utils import Coordinate
from boat import Boat
from land import Land
from weather import Weather


ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"


class Node:
    def __init__(self, coord:Coordinate, elapsed=None, parent:Optional[Self]=None):
        self.coord = coord
        self.elapsed = elapsed
        self.parent = parent


class Environment:
    def __init__(self, land_filepath, boat_filepath,
                 weather_dirpath, weather_type,
                 lon_range, lat_range,
                 datecycle:datetime, tf=24*10, dt=12, min_dist=1.0):
        '''
        weather_type: "forecast"|"historical"
        '''
        self.lon_range = lon_range
        self.lat_range = lat_range
        self.tf = tf
        self.dt = dt
        self.min_dist = min_dist
        self.land = Land(land_filepath)
        self.weather = Weather(weather_dirpath, weather_type, lon_range, lat_range, datecycle, tf, dt)
        self.boat = Boat(boat_filepath)
    

    def is_collision(self, coord1:Coordinate, coord2:Coordinate):
        line = geometry.LineString([coord1.point(), coord2.point()])
        return any(line.within(poly) for poly in self.land.polys)
    

    def time_route(self, route):
        t = 0
        for i in range(len(route)-1):
            t += self.time_to_go(Node(Coordinate(*route[i]), elapsed=t), Node(Coordinate(*route[i+1])))
        return t
    

    def distance_route(self, route):
        d = 0
        for i in range(len(route)-1):
            d += utils.haversine_distance(Coordinate(*route[i]), Coordinate(*route[i+1]))
        return d
    

    def time_to_go(self, node_start:Node, node_end:Node):
        dist = utils.haversine_distance(node_start.coord, node_end.coord)
        n = max(2, int(dist / self.min_dist))
        segments = np.linspace(node_start.coord.point(), node_end.coord.point(), n)
        
        t = node_start.elapsed
        for i in range(n-1):
            if t >= self.tf-24 or np.isnan(t): return np.inf
            t += self.time_segment(t, Coordinate(*segments[i]), Coordinate(*segments[i+1]))

        return t - node_start.elapsed
    

    def time_segment(self, elapsed, coord_start:Coordinate, coord_end:Coordinate):
        boat_speed = self.calc_boat_speed(elapsed, coord_start, coord_end)
        segment_distance = utils.haversine_distance(coord_start, coord_end)
        if boat_speed < 1e-3:
            segment_time = np.inf
        else:
            segment_time = segment_distance / boat_speed
        return segment_time


    def calc_boat_speed(self, elapsed, coord_start:Coordinate, coord_end:Coordinate):
        boat_dir = utils.angle(coord_start, coord_end)
        wind_speed, wind_dir = self.weather.calc_ws_wdir((elapsed, coord_start.lon, coord_start.lat))
        twa = utils.wrap2pi(boat_dir - wind_dir - np.pi)
        boat_speed = self.boat.calc_bsp((twa, wind_speed))
        return boat_speed