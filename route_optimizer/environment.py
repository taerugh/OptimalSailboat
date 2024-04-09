"""
Environment for route optimizer
"""

import os
import numpy as np
import shapefile as shp
import shapely.geometry as geometry
import xarray as xr
from scipy.interpolate import LinearNDInterpolator

import utils
from utils import Node

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"


class Obstacle:
    def __init__(self, shp_filepath):
        self.shapes, self.points, self.polys = self.unpack_obstacle_data(shp_filepath)

    @staticmethod
    def unpack_obstacle_data(shp_filepath):
        land_sf = shp.Reader(shp_filepath)

        land_shapes = land_sf.shapes()
        land_points = []
        land_polys = []
        for shape in land_shapes:
            shape_points = np.array(shape.points)
            shape_intervals = list(shape.parts) + [len(shape.points)]
            for (i, j) in zip(shape_intervals[:-1], shape_intervals[1:]):
                land_points.append(shape_points[i:j])
                land_polys.append(geometry.Polygon(land_points[-1]))
        
        return land_shapes, land_points, land_polys


class Weather:
    def __init__(self, grib_filepath, lon_range, lat_range):
        self.df, self.calc_uv, self.calc_ws = self.unpack_weather_data(grib_filepath, lon_range, lat_range)
    
    @staticmethod
    def unpack_weather_data(grib_filepath, lon_range, lat_range):
        weather_ds = xr.open_dataset(grib_filepath, engine='cfgrib')
        weather_df = weather_ds.to_dataframe()

        # Remove sequences
        weather_df = weather_df.loc[weather_df.index.get_level_values('orderedSequenceData') == 1].droplevel('orderedSequenceData')

        # Remove step, surface, valid_time columns
        weather_df.drop(columns=['step', 'surface', 'valid_time'], inplace=True)

        # Make lat/lon columns
        weather_df.reset_index(level=['latitude', 'longitude'], inplace=True)

        # Wrap lon to [-180, 180]
        weather_df['longitude'] = weather_df['longitude'].map(lambda lon: (lon - 360) if (lon > 180) else lon)

        # Bound lat/lon
        lat_filter = (weather_df['latitude'] >= lat_range[0]) & (weather_df['latitude'] <= lat_range[1])
        lon_filter = (weather_df['longitude'] >= lon_range[0]) & (weather_df['longitude'] <= lon_range[1])
        weather_df = weather_df.loc[lat_filter & lon_filter]

        # Convert velocities from m/s to knots
        weather_df['u'] = utils.mps2knot(weather_df['u'])
        weather_df['v'] = utils.mps2knot(weather_df['v'])
        weather_df['ws'] = utils.mps2knot(weather_df['ws'])

        # Convert angles from deg to rad
        weather_df['wdir'] = np.deg2rad(weather_df['wdir'])

        # Sort
        weather_df.sort_values(by=['longitude', 'latitude'], ascending=[True, True], inplace=True)

        ### functions
        lons = weather_df['longitude'].to_numpy()
        lats = weather_df['latitude'].to_numpy()
        us = weather_df['u'].fillna(0).to_numpy()
        vs = weather_df['v'].fillna(0).to_numpy()
        # wdirs = weather_df['wdir'].fillna(0).to_numpy()
        wss = weather_df['ws'].fillna(0).to_numpy()

        # wind velocity (u,v) as a function of location (lon, lat)
        calc_uv = LinearNDInterpolator(np.array([lons, lats]).T, np.array([us, vs]).T)
        # calc_wdir = LinearNDInterpolator(np.array([lons, lats]).T, wdirs)
        calc_ws = LinearNDInterpolator(np.array([lons, lats]).T, wss)

        return weather_df, calc_uv, calc_ws


class Boat:
    def __init__(self, csv_filepath):
        self.calc_bsp = self.unpack_boat_data(csv_filepath)

    @staticmethod
    def unpack_boat_data(csv_filepath):
        polar_data = np.loadtxt(csv_filepath, delimiter=',', skiprows=1)
        twas, twss, bsps = polar_data.T

        # BSP as a function of TWA and TWS
        calc_bsp = LinearNDInterpolator(np.array([twas, twss]).T, bsps)

        return calc_bsp


class Env:
    def __init__(self, land_filepath, weather_filepath, boat_filepath, lon_range, lat_range):
        self.lon_range = lon_range
        self.lat_range = lat_range
        self.land = Obstacle(land_filepath)
        self.weather = Weather(weather_filepath, lon_range, lat_range)
        self.boat = Boat(boat_filepath)
    
    def is_collision(self, node1:Node, node2:Node):
        line = geometry.LineString([node1.point(), node2.point()])
        return any(line.within(poly) for poly in self.land.polys)
    
    def time_route(self, route):
        t = 0
        for i in range(len(route)-1):
            t += self.time_to_go(Node(route[i]), Node(route[i+1]))
        return t
    
    def distance_route(self, route):
        d = 0
        for i in range(len(route)-1):
            d += utils.haversine_distance(Node(route[i]), Node(route[i+1]))
        return d
    
    def time_to_go(self, node_start:Node, node_end:Node, min_dist=10.0):
        dist = utils.haversine_distance(node_start, node_end)
        n = max(2, int(dist / min_dist))
        segments = np.linspace(node_start.point(), node_end.point(), n)
        time = np.sum([self.time_segment(Node(segments[i]), Node(segments[i+1])) for i in range(n-1)])
        return time
    
    def time_segment(self, node_start:Node, node_end:Node):
        boat_speed = self.boat_speed(node_start, node_end)
        segment_distance = utils.haversine_distance(node_start, node_end)
        if boat_speed < 1e-3:
            segment_time = np.inf
        else:
            segment_time = segment_distance / boat_speed
        return segment_time

    def boat_speed(self, node_start:Node, node_end:Node):
        boat_dir = utils.angle(node_start, node_end)
        wind_speed = self.weather.calc_ws(*node_start.point())
        wind_dir = np.arctan2(*self.weather.calc_uv(*node_start.point())[::-1])
        twa = utils.wrap2pi(boat_dir - wind_dir - np.pi)
        boat_speed = self.boat.calc_bsp(twa, wind_speed)
        return boat_speed