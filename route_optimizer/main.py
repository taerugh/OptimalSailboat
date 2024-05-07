import os
from datetime import datetime

from rrt_star import RRT_Star
from utils import Coordinate, PortPair

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"

# Ranges
atlantic_lon_range = (-90, 10)
atlantic_lat_range = (20, 52)
pacific_lon_range = (-180, 180)
pacific_lat_range = (10, 60)
west_pacific_lon_range = (-160, -116)
west_pacific_lat_range = (16, 40)
california_lon_range = (-126, -116)
california_lat_range = (32, 40)

# Ports
p_lehavre = Coordinate(0.0, 50.1)
p_englishchannel = Coordinate(-6.0, 49.0)
p_newyork = Coordinate(-73.0, 40.0)
p_miami = Coordinate(-79.75, 26.0)
p_stlucie = Coordinate(-79.75, 27.5)
p_sanfrancisco = Coordinate(-122.75, 37.75)
p_losangeles = Coordinate(-118.5, 33.75)
p_honolulu = Coordinate(-157.5, 21.5)

# Port Pairs
lasf = PortPair('Los Angeles to San Francisco', 'lasf', p_losangeles, p_sanfrancisco, california_lon_range, california_lat_range)
sfla = PortPair('San Francisco to Los Angeles', 'sfla', p_sanfrancisco, p_losangeles, california_lon_range, california_lat_range)
nyec = PortPair('New York to English Channel', 'nyec', p_newyork, p_englishchannel, atlantic_lon_range, atlantic_lat_range)
ecny = PortPair('English Channel to New York', 'ecny', p_englishchannel, p_newyork, atlantic_lon_range, atlantic_lat_range)

# Environment
land_filepath = ROOT_DIR + 'data/basemaps/ne_50m_land.shp'
boat_filepath = ROOT_DIR + 'data/polars/Sunfast40.pol'
weather_dirpath = ROOT_DIR + 'data/weather/'


def main():
    datecycle = datetime(2024, 4, 6, 6)

    for portpair in [lasf, sfla]:
        RRT_Star(land_filepath, boat_filepath, weather_dirpath, 'historical', portpair, datecycle, t_max=24*10,
                step_len=500.0, search_radius=500.0, goal_sample_rate=0.10, iter_max=6000,
                verbose=True, log=True).planning()


if __name__ == '__main__':
    main()