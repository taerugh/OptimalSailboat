import os
from datetime import datetime

from rrt_star import RRT_Star

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"

california_lon_range = (-126, -116)
california_lat_range = (32, 40)
p_sanfrancisco = (-122.75, 37.75)
p_losangeles = (-118.5, 33.75)

# Environment
land_filepath = ROOT_DIR + 'data/basemaps/ne_50m_land.shp'
boat_filepath = ROOT_DIR + 'data/polars/Sunfast40.csv'


def main():
    datecycle = datetime(2024, 4, 6, 6)
    weather_filepath = ROOT_DIR + f'data/weather/forecast/processed/gfswave.{datecycle.strftime('%Y%m%d%H')}.hdf5'
    RRT_Star(land_filepath, weather_filepath, boat_filepath,
             california_lon_range, california_lat_range,
             p_losangeles, p_sanfrancisco, f'test_lasf_{datecycle.strftime('%Y%m%d%H')}',
             step_len=500.0, search_radius=500.0,
             goal_sample_rate=0.10, iter_max=1000,
             verbose=True, log=False).planning()


if __name__ == '__main__':
    main()