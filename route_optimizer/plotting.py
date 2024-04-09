"""
Plotting tools for route optimizer
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

from environment import Env, Weather, Obstacle


def plot(env:Env, title='', route=None, nodelist=None, filepath=None, show=False):
    fig, ax = plt.subplots(figsize=(16,5))
    plot_map(env, fig, ax)

    if nodelist is not None:
        plot_visited(ax, nodelist)

    if route is not None:
        plot_waypoint(ax, route[0], 'Start')
        plot_waypoint(ax, route[-1], 'Goal')
        plot_route(ax, route)

    if filepath is not None:
        plt.savefig(filepath)
        plt.close()
    if show:
        plt.show()
        plt.close()
    
    fig.suptitle(title)
    fig.tight_layout()
    
    return fig, ax


def plot_map(env:Env, fig, ax):
    setup_plot(env.lon_range, env.lat_range, ax)
    plot_windspeed(env.weather, env.lon_range, env.lat_range, fig, ax)
    plot_land(env.land, ax)
    plot_windstream(env.weather, env.lon_range, env.lat_range, ax)


def setup_plot(lon_range, lat_range, ax):
    ax.set_xlim(lon_range)
    ax.set_ylim(lat_range)
    ax.set_aspect('equal')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')


def plot_windspeed(weather:Weather, lon_range, lat_range, fig, ax):
    lons = np.linspace(*lon_range, 500)
    lats = np.linspace(*lat_range, 500)
    lons, lats = np.meshgrid(lons, lats)
    colormesh = ax.pcolormesh(lons, lats, weather.calc_ws(lons,lats), cmap='Blues', shading='gouraud', alpha = 1.0)
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05, aspect=0.9)
    fig.colorbar(colormesh, ax=ax, cax=cax, label='Wind Speed [knots]', location='right')


def plot_windstream(weather:Weather, lon_range, lat_range, ax):
    lons = np.linspace(*lon_range, 500)
    lats = np.linspace(*lat_range, 500)
    lons, lats = np.meshgrid(lons, lats)
    ax.streamplot(lons, lats, weather.calc_uv(lons,lats)[:,:,0], weather.calc_uv(lons,lats)[:,:,1], color='k', density=2, linewidth=0.5)


def plot_land(land:Obstacle, ax):
    for shape_points in land.points:
        ax.fill(*shape_points.T, color='slategrey')
        ax.plot(*shape_points.T, color='darkslategrey')


def plot_visited(ax, nodelist):
    for node in nodelist:
        if node.parent:
            ax.plot([node.parent.lon, node.lon], [node.parent.lat, node.lat], color='lightcoral', linewidth=1)


def plot_route(ax, route):
    if len(route) != 0:
        ax.plot(*route.T, color='red', linewidth=2)


def plot_waypoint(ax, p, label):
    ax.scatter(*p, color='red', marker='*', s=50)
    ax.annotate(label, p, xytext=(0,10), textcoords='offset pixels', color='darkred')