'''
https://nomads.ncep.noaa.gov/gribfilter.php?ds=gfswave
'''

import os
import sys
import pathlib
import time
from datetime import datetime, timedelta
from urllib.request import urlretrieve

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"
sys.path.append(ROOT_DIR)

from route_optimizer import utils


def pull_weather_file(datecycle:datetime, step, dirpath):
       '''
       step = '001' # forecast step (1 hour intervals)
       '''
       date = datecycle.strftime('%Y%m%d')
       cycle = datecycle.strftime('%H')
       url = ('https://nomads.ncep.noaa.gov/cgi-bin/filter_gfswave.pl'
              f'?dir=%2Fgfs.{date}%2F{cycle}%2Fwave%2Fgridded'
              f'&file=gfswave.t{cycle}z.global.0p16.f{step}.grib2'
              '&all_var=on' + '&lev_surface=on')
       filename = f'gfswave.{datecycle.strftime('%Y%m%d%H')}.{step}.grib2'
       pathlib.Path(dirpath).mkdir(exist_ok=True)

       if not os.path.isfile(dirpath+filename):
              urlretrieve(url, dirpath+filename)
              time.sleep(10.0)


def scrape_historical(datecycle:datetime, tf=384, dt=12):
       dirpath = ROOT_DIR + f'data/weather/historical/raw/'
       for t in range(0, tf, dt):
              datecycle_t = datecycle + timedelta(hours=t)
              pull_weather_file(datecycle_t, '000', dirpath)


def scrape_forecast(datecycle:datetime, tf=384, dt=12):
       steps = [str(k).zfill(3) for k in range(0, tf, dt)]
       dirpath = ROOT_DIR + f'data/weather/forecast/raw/gfswave.{datecycle.strftime('%Y%m%d%H')}/'
       for step in steps:
              pull_weather_file(datecycle, step, dirpath)


def main():
       datecycles = [datetime(2024, 4, day, 6) for day in range(11,18)]
       tf = 24*12

       for datecycle in datecycles:
              #* Historical
              # print(f'Scraping {datecycle.strftime('%Y%m%d%H')} historical data...', end=' ', flush=True)
              # scrape_historical(datecycle, tf=tf)
              # print('Done!')

              # #* Forecast
              print(f'Scraping {datecycle.strftime('%Y%m%d%H')} forecast data...', end=' ', flush=True)
              scrape_forecast(datecycle)
              print('Done!')


if __name__ == '__main__':
       main()