"""
Utilities for route optimizer
"""

import numpy as np

#* Constants
R = 3443.92 # earth's radius [nmi]


#* Classes
class Coordinate:
    def __init__(self, lon, lat):
        self.lon = lon
        self.lat = lat
    
    def point(self):
        return (self.lon, self.lat)


class PortPair:
    def __init__(self, longname, shortname, p_start:Coordinate, p_end:Coordinate, lon_range, lat_range):
        self.longname = longname
        self.shortname = shortname
        self.p_start = p_start
        self.p_end = p_end
        self.lon_range = lon_range
        self.lat_range = lat_range


#* Functions
def haversine_distance(coord1:Coordinate, coord2:Coordinate):
    dlat = np.deg2rad(coord2.lat - coord1.lat)
    dlon = np.deg2rad(coord2.lon - coord1.lon)
    a = np.sin(dlat/2)**2 + (np.cos(np.deg2rad(coord1.lat)) * np.cos(np.deg2rad(coord2.lat)) * np.sin(dlon/2)**2)
    c = 2*np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d = R*c
    return d

def euclidean_distance(coord1:Coordinate, coord2:Coordinate):
    return np.hypot(coord2.lon - coord1.lon, coord2.lat - coord1.lat)

def angle(coord1:Coordinate, coord2:Coordinate):
    ''' Angle from coord1 to coord2 '''
    return np.arctan2(coord2.lat - coord1.lat, coord2.lon - coord1.lon)

def transform_coord(coord_start:Coordinate, d, theta):
    dlon = d*np.cos(theta) / (R*np.cos(np.deg2rad(coord_start.lat)))
    dlat = d*np.sin(theta) / R
    return Coordinate(coord_start.lon + dlon, coord_start.lat + dlat)

def mps2knot(v_mps):
    ''' Convert speed from m/s to knots '''
    v_knot = 1.94384*v_mps
    return v_knot

def wrap2pi(theta):
    return (theta + np.pi)%(2*np.pi) - np.pi

def second2hour(seconds):
    return seconds / 3600.0