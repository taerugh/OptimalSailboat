import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


def car2pol(x, y):
    """ Converts from cartesian (x,y) to polar (r,θ) coordinates. """
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    return r, theta


def wrap2pi(theta):
    """ Wraps an angle (in radians) to the range [-π, π]. """
    return theta - 2*np.pi*np.floor((theta+np.pi)/(2*np.pi))


def true_wind(lon, lat):
    """ true wind velocity (vector field) """
    u = 1.
    v = 0.
    return u, v


def dynamics(x, u, p):
    ''' Nonlinear dynamics of the sailboat model. '''
    lon, lat, theta, v, omega = x
    d_r, d_s = u
    
    # true wind
    u_tw, v_tw = true_wind(lon, lat)
    a_tw, psi_tw = car2pol(u_tw, v_tw)
    
    # apparent wind
    u_aw = a_tw * np.cos(psi_tw - theta) - v
    v_aw = a_tw * np.sin(psi_tw - theta)
    a_aw, psi_aw = car2pol(u_aw, v_aw)
    
    # forces
    g_s = p[3] * a_aw * np.sin(d_s - psi_aw)
    g_r = p[4] * v**2 * np.sin(d_r)

    # dynamics
    x_dot = np.array([v*np.cos(theta) + p[0]*a_tw*np.cos(psi_tw),
                      v*np.sin(theta) + p[0]*a_tw*np.sin(psi_tw),
                      omega,
                      (g_s*np.sin(d_s) - g_r*p[10]*np.sin(d_r) - p[1]*v**2)/(p[8]),
                      (g_s*(p[5] - p[6]*np.cos(d_s)) - g_r*p[7]*np.cos(d_r) - p[2]*omega*v)/(p[9])])

    return x_dot


def rk4(x, u, p, dt):
    """ Runge-Kutta (4th order) explicit integration. """
    k1 = dt*dynamics(x, u, p)
    k2 = dt*dynamics(x + k1/2, u, p)
    k3 = dt*dynamics(x + k2/2, u, p)
    k4 = dt*dynamics(x + k3, u, p)

    x_next = x + (k1 + 2*k2 + 2*k3 + k4)/6
    x_next[3] = wrap2pi(x_next[3])
    
    return x_next


def hermite_simpson(x1, x2, u, p, dt):
    """ Hermite-Simpson implicit integration. """
    x1_dot = dynamics(x1, u, p)
    x2_dot = dynamics(x2, u, p)

    x_mid = (x1 + x2)/2 + dt*(x1_dot - x2_dot)/8

    return x1 + dt*(x1_dot + 4*dynamics(x_mid, u, p) + x2_dot)/6 - x2


def plot_boat(ax, x, u, p):
        lon, lat, theta, v, omega = x
        d_r, d_s = u
        k1 = 0.4 # boat width scale factor
        k3 = 0.3 # rudder scale factor

        points = np.array([[lon + (p[5]+p[6])*np.cos(theta), lat + (p[5]+p[6])*np.sin(theta)],
                           [lon + k1*p[6]*np.sin(theta) + p[5]*np.cos(theta), lat - k1*p[6]*np.cos(theta) + p[5]*np.sin(theta)],
                           [lon + k1*p[6]*np.sin(theta) - p[6]*np.cos(theta), lat - k1*p[6]*np.cos(theta) - p[6]*np.sin(theta)],
                           [lon - k1*p[6]*np.sin(theta) - p[6]*np.cos(theta), lat + k1*p[6]*np.cos(theta) - p[6]*np.sin(theta)],
                           [lon - k1*p[6]*np.sin(theta) + p[5]*np.cos(theta), lat + k1*p[6]*np.cos(theta) + p[5]*np.sin(theta)]])

        ax.fill(points[:,0], points[:,1], color='k', alpha=0.2) # boat
        ax.arrow(lon-p[6]*np.cos(theta), lat-p[6]*np.sin(theta), -k3*p[6]*np.cos(theta+d_r), -k3*p[6]*np.sin(theta+d_r)) # rudder
        ax.arrow(lon+p[5]*np.cos(theta), lat+p[5]*np.sin(theta), -2*p[5]*np.cos(theta+d_s), -2*p[5]*np.sin(theta+d_s)) # sail


if __name__ == '__main__':
    p = np.array([0.03, 40, 6000, 200, 1500, 0.5, 0.5, 2, 300, 400, 0.2])
    x0 = np.array([0.0, 0.0, np.pi/2, 0.0, 0.0])
    xg = np.array([0.0, 10.0, np.pi/2, 0.0, 0.0])
    u = np.array([0, np.pi/16])
    dt = 0.05
    tf = 10

    N = int(tf/dt)
    nx = 5
    nu = 2
    nz = (N-1) * nu + N * nx
    idx_x = [range(i*(nx+nu), i*(nx+nu)+nx) for i in range(N)]
    idx_u = [range(i*(nx+nu)+nx, i*(nx+nu)+nx+nu) for i in range(N-1)]

    Q = np.diag([1., 1., 0., 0., 0.])
    R = 0.1*np.eye(nu)
    Qf = 10*Q

    z0 = 0.001*np.random.randn(nz)
    z0[idx_x[0]] = x0

    for i in range(N-1):
        z0[idx_u[i]] = u
        z0[idx_x[i+1]] = rk4(z0[idx_x[i]], z0[idx_u[i]], p, dt)

    def cost(z):
        J = 0 
        for i in range(N-1):
            xi = z[idx_x[i]]
            ui = z[idx_u[i]]
            J += (xi - xg).T @ Q @ (xi - xg)
            J += ui.T @ R @ ui

        xf = z[idx_x[N-1]]
        J += (xf - xg).T @ Qf @ (xf - xg)
        
        return J
    
    def dyncon(z):
        c = np.zeros(nx*(N-1))
        for i in range(N-1):
            xi = z[idx_x[i]]
            ui = z[idx_u[i]]
            xip1 = z[idx_x[i+1]]
            c[nx*i:nx*(i+1)] = hermite_simpson(xi, xip1, ui, p, dt)
            # c[idx.c[i]] = xip1 - rk4(xi, ui, p, dt)
        return c
    
    dynamics_constraint = {
        'type': 'eq',
        'fun': dyncon
    }

    ic_constraint = {
        'type': 'eq',
        'fun': lambda z: z[idx_x[0]] - x0
    }

    def callback(z):
        print(cost(z))

    res = minimize(cost, z0, constraints=[dynamics_constraint, ic_constraint], method='SLSQP', callback=callback, options={'maxiter': 3})
    print(res)


    # fig, ax = plt.subplots()
    # plot_boat(ax, x, u, p)
    # ax.set_ylim([-20, 20])
    # ax.set_xlim([-20, 20])

    # for i in range(int(tf/dt)):
    #     ax.cla()
    #     plot_boat(ax, x, u, p)
    #     ax.set_ylim([-20, 20])
    #     ax.set_xlim([-20, 20])
    #     plt.pause(dt)
    #     x = rk4(x, u, p, dt)