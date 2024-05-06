import cvxpy as cp
import numpy as np

def mpc(A, B, X_ref_window, U_ref_window, xic, xg, u_min, u_max, N_mpc):
    nx,nu = B.shape
    
    Q = 0.01 * np.diag(np.ones(nx))
    R = 0.01 * np.diag(np.ones(nu))

    X = cp.Variable((nx, N_mpc))
    U = cp.Variable((nu, N_mpc-1))

    cost = 0.0
    for k in range(N_mpc-1):
        x = X[:, k]
        u = U[:, k]
        state_cost = cp.quad_form((x - X_ref_window[:, k]), Q)
        if U_ref_window is not None:
            input_cost = cp.quad_form(u - U_ref_window[:, k], R)
        else:
            input_cost = cp.quad_form(u, R)

        cost += 0.5 * state_cost + 0.5 * input_cost
    
    cost += 0.5 * cp.quad_form((X[:, N_mpc-1] - X_ref_window[:, N_mpc-1]), Q)

    constraints = [X[:, 0] == xic]

    for k in range(N_mpc-1):
        constraints.append(X[:, k+1] == A @ X[:, k] + B @ U[:, k])
        #constraints.append(u_min <= U[:, k])
        #constraints.append(U[:, k] <= u_max)

    constraints.append(X[1, N_mpc-1] == xg[1])
    
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.ECOS, verbose=True, abstol=1e-8, reltol=1e-8, feastol=1e-8)
    
    return U.value