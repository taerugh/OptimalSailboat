'''
Land object for route_optimizer
author: @taerugh
'''

import numpy as np
import shapefile as shp
import shapely.geometry as geometry


class Land:
    def __init__(self, shp_filepath):
        self.shapes, self.points, self.polys = self.unpack_land_data(shp_filepath)


    @staticmethod
    def unpack_land_data(shp_filepath):
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