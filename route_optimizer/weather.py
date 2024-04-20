'''
Weather object for route_optimizer
author: @taerugh
'''

import os
import numpy as np
import pandas as pd
import xarray as xr
from datetime import datetime, timedelta
from scipy.interpolate import RegularGridInterpolator

import utils


class Weather:
    def __init__(self, weather_dirpath, weather_type, datecycle:datetime, lon_range, lat_range, tf, dt):
        self.df, self.calc_u, self.calc_v = self.unpack_weather_data(weather_dirpath, weather_type, datecycle, lon_range, lat_range, tf, dt)


    def calc_ws(self, xi):
        ''' xi: [t, lon, lat] '''
        u = self.calc_u(xi)
        v = self.calc_v(xi)
        ws = np.hypot(u, v)
        return ws


    def calc_wdir(self, xi):
        ''' xi: [t, lon, lat] '''
        u = self.calc_u(xi)
        v = self.calc_v(xi)
        wdir = np.arctan2(v, u)
        return wdir
    

    def calc_ws_wdir(self, xi):
        ''' xi: [t, lon, lat] '''
        u = self.calc_u(xi)
        v = self.calc_v(xi)
        ws = np.hypot(u, v)
        wdir = np.arctan2(v, u)
        return ws, wdir
    

    def unpack_weather_data(self, weather_dirpath, weather_type, datecycle:datetime, lon_range, lat_range, tf, dt):
        '''
        type: 'forecast' or 'historical'
        '''
        if weather_type not in {'forecast', 'historical'}:
            raise ValueError('Invalid weather type')

        # Check if processed weather data exists, otherwise process it
        processed_weather_filepath = os.path.join(weather_dirpath, f'{weather_type}/processed/gfswave.{datecycle.strftime('%Y%m%d%H')}.hdf5')
        if os.path.exists(processed_weather_filepath):
            weather_df = pd.read_hdf(processed_weather_filepath, key='df')
        else:
            weather_df = self.preprocess_weather_data(weather_dirpath, processed_weather_filepath, weather_type, datecycle, tf, dt)

        # Bound lat/lon
        lats = weather_df['latitude'].unique()
        lons = weather_df['longitude'].unique()
        min_lat = np.max(lats[lats < lat_range[0]])
        max_lat = np.min(lats[lats > lat_range[1]])
        min_lon = np.max(lons[lons < lon_range[0]])
        max_lon = np.min(lons[lons > lon_range[1]])
        weather_df = weather_df.loc[(weather_df['latitude'] >= min_lat) & (weather_df['latitude'] <= max_lat) &
                                    (weather_df['longitude'] >= min_lon) & (weather_df['longitude'] <= max_lon)]

        ### functions
        elapsed = weather_df['elapsed'].unique()
        lons = weather_df['longitude'].unique()
        lats = weather_df['latitude'].unique()
        us = weather_df['u'].fillna(0).to_numpy()
        vs = weather_df['v'].fillna(0).to_numpy()

        # wind velocity (u,v) as a function of time and location (step, lon, lat)
        calc_u = RegularGridInterpolator((elapsed, lons, lats), us.reshape(len(elapsed), len(lons), len(lats)))
        calc_v = RegularGridInterpolator((elapsed, lons, lats), vs.reshape(len(elapsed), len(lons), len(lats)))

        return weather_df, calc_u, calc_v
    

    @staticmethod
    def preprocess_weather_data(weather_dirpath, processed_weather_filepath, weather_type, datecycle:datetime, tf, dt):
        weather_filepaths = []
        match weather_type:
            case 'forecast':
                for t in range(0, tf, dt):
                    weather_filepaths.append(os.path.join(weather_dirpath, f'forecast/raw/gfswave.{datecycle.strftime('%Y%m%d%H')}/gfswave.{datecycle.strftime('%Y%m%d%H')}.{str(t).zfill(3)}.grib2'))
            case 'historical':
                for t in range(0, tf, dt):
                    datecycle_t = datecycle + timedelta(hours=t)
                    weather_filepaths.append(os.path.join(weather_dirpath, f'historical/raw/gfswave.{datecycle_t.strftime('%Y%m%d%H')}.000.grib2'))
            case default:
                raise ValueError('Invalid weather type')

        # Unpack weather data
        weather_df = pd.concat([xr.open_dataset(weather_filepath, engine='cfgrib', indexpath='').to_dataframe() for weather_filepath in weather_filepaths])

        # Change lat/lon from indices to columns
        weather_df.reset_index(level=['latitude', 'longitude'], inplace=True)

        # Wrap lon to [-180, 180]
        weather_df['longitude'] = weather_df['longitude'].map(lambda lon: (lon - 360) if (lon > 180) else lon)

        # Change step to hours
        weather_df['elapsed'] = [utils.second2hour(elapsed.total_seconds()) for elapsed in (weather_df['valid_time'] - datecycle)]

        # Convert velocities from m/s to knots
        weather_df['u'] = utils.mps2knot(weather_df['u'])
        weather_df['v'] = utils.mps2knot(weather_df['v'])

        # Order columns and drop unused
        weather_df = weather_df[['elapsed', 'longitude', 'latitude', 'u', 'v', 'ws']]

        # Sort
        weather_df.sort_values(by=['elapsed', 'longitude', 'latitude'], ascending=[True, True, True], inplace=True)

        # Save processed weather data
        weather_df.to_hdf(processed_weather_filepath, key='df')

        return weather_df