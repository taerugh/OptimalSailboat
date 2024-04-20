"""
Plotting tools for route optimizer
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import Figure, Axes
from matplotlib.animation import FuncAnimation
from mpl_toolkits.axes_grid1 import make_axes_locatable

import utils
from utils import Coordinate
from environment import Environment, Node
from weather import Weather
from land import Land


def plot(env:Environment, t=None, title='', route=None, nodelist=None, filepath=None, show=False):
    fig, ax = plt.subplots()
    setup_plot(ax, env.lon_range, env.lat_range)
    if t is not None: plot_wind(fig, ax, env, t)
    plot_land(ax, env.land)
    if nodelist is not None: plot_visited(ax, nodelist)

    if route is not None:
        plot_waypoint(ax, Coordinate(*route[0]), 'Start')
        plot_waypoint(ax, Coordinate(*route[-1]), 'Goal')
        plot_route(ax, route)

    fig.suptitle(title)
    fig.tight_layout()

    if filepath is not None: plt.savefig(filepath)
    if show: plt.show()
    
    return fig, ax


def animate(env:Environment, tf, dt, title='', route=None, filepath=None, v_range=None, min_dist=10.0):
    if v_range is None: v_range = (env.weather.df['ws'].min(),env.weather.df['ws'].max())
    fig, ax = plt.subplots()
    setup_plot(ax, env.lon_range, env.lat_range)
    plot_wind(fig, ax, env, 0, v_range=v_range, colorbar=True)
    plot_land(ax, env.land)
    fig.suptitle(title)
    fig.tight_layout()

    if route is not None:
        route_segments = []
        for i in range(len(route)-1):
            dist = utils.haversine_distance(Coordinate(*route[i]), Coordinate(*route[i+1]))
            n = max(2, int(dist / min_dist))
            route_segments.extend(np.linspace(route[i], route[i+1], n))
        route_segments = np.array(route_segments)

    def func(t):
        ax.cla()
        setup_plot(ax, env.lon_range, env.lat_range)
        plot_wind(fig, ax, env, t, v_range=v_range, colorbar=False)
        plot_land(ax, env.land)
        if route is not None:
            plot_waypoint(ax, Coordinate(*route[0]), 'Start')
            plot_waypoint(ax, Coordinate(*route[-1]), 'Goal')
            plot_dynamicroute(ax, env, route_segments, t)
        ax.set_title(f'elapsed time: {t} hours')

    ani = FuncAnimation(fig, func, frames=np.arange(0, tf, dt))

    if filepath is not None: ani.save(filepath, fps=10)


def animate_winddelta(env1:Environment, env2:Environment, tf, dt, title='', filepath=None):
    alldeltas = np.hypot(env1.weather.df['u'] - env2.weather.df['u'], env1.weather.df['v'] - env2.weather.df['v'])
    v_range = (np.min(alldeltas), np.max(alldeltas))
    fig, ax = plt.subplots()
    setup_plot(ax, env1.lon_range, env1.lat_range)
    plot_winddelta(fig, ax, env1.weather, env2.weather, 0, env1.lon_range, env1.lat_range, v_range=v_range)
    plot_land(ax, env1.land)
    fig.suptitle(title)

    def func(t):
        ax.cla()
        setup_plot(ax, env1.lon_range, env1.lat_range)
        plot_winddelta(fig, ax, env1.weather, env2.weather, t, env1.lon_range, env1.lat_range, v_range=v_range, colorbar=False)
        plot_land(ax, env1.land)
        ax.set_title(f'elapsed time: {t} hours')

    ani = FuncAnimation(fig, func, frames=np.arange(0, tf, dt))

    if filepath is not None:
        ani.save(filepath, fps=10)


def setup_plot(ax:Axes, lon_range, lat_range):
    ax.set_xlim(lon_range)
    ax.set_ylim(lat_range)
    ax.set_aspect('equal')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')


def plot_winddelta(fig:Figure, ax:Axes, weather1:Weather, weather2:Weather, t, lon_range, lat_range, v_range=(None,None), colorbar=True):
    lons, lats = np.linspace(*lon_range, 500), np.linspace(*lat_range, 500)
    lons, lats = np.meshgrid(lons, lats)
    ts = t*np.ones_like(lons)
    deltas = np.hypot(weather1.calc_u((ts,lons,lats)) - weather2.calc_u((ts,lons,lats)), weather1.calc_v((ts,lons,lats)) - weather2.calc_v((ts,lons,lats)))
    colormesh = ax.pcolormesh(lons, lats, deltas, cmap='Blues', shading='gouraud', vmin=v_range[0], vmax=v_range[1])
    if colorbar:
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.05, aspect=0.9)
        fig.colorbar(colormesh, ax=ax, cax=cax, label='Error Magnitude [knots]', location='right')


def plot_wind(fig:Figure, ax:Axes, env:Environment, t, v_range=(None,None), colorbar=True):
    plot_windspeed(fig, ax, env.weather, t, env.lon_range, env.lat_range, v_range=v_range, colorbar=colorbar)
    plot_windstream(ax, env.weather, t, env.lon_range, env.lat_range)


def plot_windspeed(fig:Figure, ax:Axes, weather:Weather, t, lon_range, lat_range, v_range=(None,None), colorbar=True):
    lons, lats = np.linspace(*lon_range, 500), np.linspace(*lat_range, 500)
    lons, lats = np.meshgrid(lons, lats)
    ts = t*np.ones_like(lons)
    colormesh = ax.pcolormesh(lons, lats, weather.calc_ws((ts,lons,lats)), cmap='Blues', shading='gouraud', vmin=v_range[0], vmax=v_range[1])
    if colorbar:
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.05, aspect=0.9)
        fig.colorbar(colormesh, ax=ax, cax=cax, label='Wind Speed [knots]', location='right')


def plot_windstream(ax:Axes, weather:Weather, t, lon_range, lat_range):
    lons, lats = np.linspace(*lon_range, 500), np.linspace(*lat_range, 500)
    lons, lats = np.meshgrid(lons, lats)
    ts = t*np.ones_like(lons)
    ax.streamplot(lons, lats, weather.calc_u((ts,lons,lats)), weather.calc_v((ts,lons,lats)), color='k', density=2, linewidth=0.5)


def plot_land(ax:Axes, land:Land):
    for shape_points in land.points:
        ax.fill(*shape_points.T, color='slategrey')
        ax.plot(*shape_points.T, color='darkslategrey')


def plot_visited(ax:Axes, nodelist:list[Node]):
    for node in nodelist:
        if node.parent:
            ax.plot([node.parent.coord.lon, node.coord.lon], [node.parent.coord.lat, node.coord.lat], color='lightcoral', linewidth=1)


def plot_route(ax:Axes, route):
    if len(route) == 0: return
    ax.plot(*route.T, color='red', linewidth=2)

def plot_dynamicroute(ax:Axes, env:Environment, route_segments, elapsed):
    if len(route_segments) == 0: return

    t = 0
    i = 0
    while i<len(route_segments)-1 and t<elapsed:
        t += env.time_segment(t, Coordinate(*route_segments[i]), Coordinate(*route_segments[i+1]))
        i += 1

    ax.plot(*route_segments[:i,:].T, color='red', linewidth=2)


def plot_waypoint(ax:Axes, coord:Coordinate, label):
    ax.scatter(coord.lon, coord.lat, color='red', marker='*', s=50)
    ax.annotate(label, coord.point(), xytext=(0,10), textcoords='offset pixels', color='darkred')