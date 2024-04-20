"""
Utilities for route optimizer
"""

import numpy as np


class Coordinate:
    def __init__(self, lon, lat):
        self.lon = lon
        self.lat = lat
    
    def point(self):
        return (self.lon, self.lat)


R = 3443.92 # earth's radius [nmi]
    
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


# def create_gif_from_folder(folder_path, output_path):
#     images = []
#     for filename in os.listdir(folder_path):
#         if filename.endswith(".png"):
#             image_path = os.path.join(folder_path, filename)
#             image = Image.open(image_path)
#             images.append(image)

#     output_path = os.path.join(output_path, "output.gif")
#     images[0].save(output_path, save_all=True, append_images=images[1:], optimize=False, duration=200, loop=0)
#     return output_path