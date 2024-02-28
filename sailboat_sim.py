import numpy as np
from numpy import sin, cos, arctan2, sqrt
from numpy.linalg import norm

import matplotlib.pyplot as plt
from pynput import keyboard


class Sailboat():
    def __init__(self, state0, u0, w, p, dt):
        self.state = state0
        self.u = u0
        self.w = w
        self.p = p
        self.dt = dt
        self.running = True

        listener = keyboard.Listener(on_press=self._on_press, suppress=True)
        listener.start()

        self._dr_increment = 0.1
        self._ds_increment = 0.1


    def step(self):
        k1 = self.dt*self._dynamics(self.state, self.u, self.w, self.p)
        k2 = self.dt*self._dynamics(self.state+k1/2, self.u, self.w, self.p)
        k3 = self.dt*self._dynamics(self.state+k2/2, self.u, self.w, self.p)
        k4 = self.dt*self._dynamics(self.state+k3, self.u, self.w, self.p)
        self.state += (1/6)*(k1 + 2*k2 + 2*k3 + k4)
        self.state[2] = self._wrap2pi(self.state[2])
        # self.q[3] = self._wrap2pi(self.q[3])


    def plot(self, ax):
        ax.cla()
        self._plot_boat(ax)
        self._plot_wind(ax)
        ax.set_ylim([-50, 50])
        ax.set_xlim([-50, 50])
    

    def _dynamics(self, state, u, w, p):
        x, y, theta, v, dtheta = state
        d_r, d_s = u
        a_tw, psi_tw = w

        W_aw_x = a_tw*cos(psi_tw-theta) - v
        W_aw_y = a_tw*sin(psi_tw-theta)
        a_aw = sqrt(W_aw_x**2 + W_aw_y**2)
        psi_aw = arctan2(W_aw_y, W_aw_x)

        g_s = p[3] * a_aw * sin(d_s - psi_aw)
        g_r = p[4] * v**2 * sin(d_r)

        return np.array([v*cos(theta) + p[0]*a_tw*cos(psi_tw),
                         v*sin(theta) + p[0]*a_tw*cos(psi_tw),
                         dtheta,
                         (g_s*sin(d_s) - g_r*p[10]*sin(d_r) - p[1]*v**2)/(p[8]),
                         (g_s*(p[5] - p[6]*cos(d_s)) - g_r*p[7]*cos(d_r) - p[2]*dtheta*v)/(p[9])])


    def _plot_boat(self, ax):
        x, y, theta, v, dtheta = self.state
        d_r, d_s = self.u
        k1 = 0.4 # boat width scale factor
        k3 = 0.3 # rudder scale factor

        points = np.array([[x + (self.p[6]+self.p[7])*cos(theta), y + (self.p[6]+self.p[7])*sin(theta)],
                           [x + k1*p[7]*sin(theta) + p[6]*cos(theta), y - k1*p[7]*cos(theta) + p[6]*sin(theta)],
                           [x + k1*p[7]*sin(theta) - p[7]*cos(theta), y - k1*p[7]*cos(theta) - p[7]*sin(theta)],
                           [x - k1*p[7]*sin(theta) - p[7]*cos(theta), y + k1*p[7]*cos(theta) - p[7]*sin(theta)],
                           [x - k1*p[7]*sin(theta) + p[6]*cos(theta), y + k1*p[7]*cos(theta) + p[6]*sin(theta)]])

        ax.fill(points[:,0], points[:,1], color='k', alpha=0.2) # boat
        ax.arrow(x-p[7]*cos(theta), y-p[7]*sin(theta), -k3*p[7]*cos(theta+d_r), -k3*p[7]*sin(theta+d_r)) # rudder
        ax.arrow(x+p[6]*cos(theta), y+p[6]*sin(theta), -2*p[6]*cos(theta+d_s), -2*p[6]*sin(theta+d_s)) # sail
    

    def _plot_wind(self, ax):
        a_tw, psi_tw = w
        xs,ys = np.meshgrid(np.linspace(-50,50,20), np.linspace(-50,50,20))
        dxs = a_tw*cos(psi_tw)*np.ones_like(xs)
        dys = a_tw*sin(psi_tw)*np.ones_like(ys)

        ax.quiver(xs, ys, dxs, dys, linewidth=0.05, color='#A23BEC') 
    

    def _on_press(self, key):
        try:
            if key.char == 'w':
                self.u[1] += self._ds_increment
            if key.char == 's':
                self.u[1] -= self._ds_increment
            if key.char == 'a':
                self.u[0] -= self._dr_increment
            if key.char == 'd':
                self.u[0] += self._dr_increment
        except AttributeError:
            if key == keyboard.Key.esc:
                self.running = False
    

    def _wrap2pi(self, angle):
        angle = angle - 2*np.pi*np.floor((angle+np.pi )/(2*np.pi))
        return angle


if __name__ == '__main__':
    state0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    w = np.array([1.0, 0])
    p = np.array([0.03, 40, 6000, 200, 1500, 0.5, 0.5, 2, 300, 400, 0.2])
    dt = 0.05
    sailboat = Sailboat(state0, u0, w, p, dt)
    fig, ax = plt.subplots()

    i=0
    while sailboat.running:
        sailboat.step()
        sailboat.plot(ax)
        plt.pause(dt)
        if i%100 == 0:
            print(f't = {dt*i:0.2f} s')
            print('\t' + f'u = {sailboat.u}')
            print('\t' + f'x = {sailboat.state}')
        i+=1