"""
Utilities for route optimizer
"""

import numpy as np

from PIL import Image
import os


class Node:
    def __init__(self, n, parent=None, cost=None):
        self.lon = n[0]
        self.lat = n[1]
        self.parent = parent
        self.cost = cost

    def point(self):
        return (self.lon, self.lat)

R = 3443.92 # earth's radius [nmi]
    
def haversine_distance(node1:Node, node2:Node):
    dlat = np.deg2rad(node2.lat - node1.lat)
    dlon = np.deg2rad(node2.lon - node1.lon)
    a = np.sin(dlat/2)**2 + (np.cos(np.deg2rad(node1.lat)) * np.cos(np.deg2rad(node2.lat)) * np.sin(dlon/2)**2)
    c = 2*np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d = R*c
    return d

def euclidean_distance(node1:Node, node2:Node):
    return np.hypot(node2.lon - node1.lon, node2.lat - node1.lat)

def angle(node_start:Node, node_end:Node):
    return np.arctan2(node_end.lat - node_start.lat, node_end.lon - node_start.lon)

def transform_node(node_start:Node, d, theta):
    dlon = d*np.cos(theta) / (R*np.cos(np.deg2rad(node_start.lat)))
    dlat = d*np.sin(theta) / R
    return Node((node_start.lon + dlon, node_start.lat + dlat))

def mps2knot(v_mps):
    ''' Convert speed from m/s to knots '''
    v_knot = 1.94384*v_mps
    return v_knot

def wrap2pi(theta):
    return (theta + np.pi)%(2*np.pi) - np.pi


def create_gif_from_folder(folder_path, output_path):
    images = []
    for filename in os.listdir(folder_path):
        if filename.endswith(".png"):
            image_path = os.path.join(folder_path, filename)
            image = Image.open(image_path)
            images.append(image)

    output_path = os.path.join(output_path, "output.gif")
    images[0].save(output_path, save_all=True, append_images=images[1:], optimize=False, duration=200, loop=0)
    return output_path