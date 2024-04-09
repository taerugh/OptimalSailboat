import os

from rrt_star import RRT_Star

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../"


def main():
    lon_range = (-90, 10)
    lat_range = (20, 52)
    p_englishchannel = (-6.0, 49.0)
    p_newyork = (-73.0, 40.0)

    land_filepath = ROOT_DIR + 'data/basemaps/ne_50m_land.shp'
    boat_filepath = ROOT_DIR + 'data/polars/Sunfast40.csv'
    weather_filepath = ROOT_DIR + 'data/weather_log/gfswave.t18z.global.0p16.f000.20240326.grib2'

    rrt_star = RRT_Star(land_filepath, weather_filepath, boat_filepath,
                        lon_range, lat_range,
                        p_englishchannel, p_newyork, f'test',
                        step_len=500.0, search_radius=500.0,
                        goal_sample_rate=0.10, iter_max=3000,
                        verbose=True, log=False)
    rrt_star.planning()


if __name__ == '__main__':
    main()