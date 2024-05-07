'''
Boat object for route_optimizer
author: @taerugh
'''

import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator


class Boat:
    def __init__(self, pol_filepath):
        self.polar_df, self.calc_bsp = self.unpack_boat_data(pol_filepath)


    @staticmethod
    def unpack_boat_data(pol_filepath) -> tuple[pd.DataFrame, RegularGridInterpolator]:
        polar_df = pd.read_csv(pol_filepath, delimiter=',')
        polar_df.rename(columns={'TWA\\TWS': 'TWA'}, inplace=True)

        # Add mirrored angles
        polar_mirror_df = polar_df.copy()
        polar_mirror_df['TWA'] = -polar_mirror_df['TWA']
        polar_df = polar_df.merge(polar_mirror_df, how='outer')

        # Convert to radians
        polar_df['TWA'] = np.deg2rad(polar_df['TWA'])

        # Set TWA to the DataFrame index
        polar_df.set_index('TWA', inplace=True)

        twas = polar_df.index.to_numpy(dtype=float)
        twss = polar_df.columns.to_numpy(dtype=float)
        bsps = polar_df.to_numpy(dtype=float)

        # BSP as a function of TWA and TWS
        calc_bsp = RegularGridInterpolator((twas, twss), bsps)

        return polar_df, calc_bsp