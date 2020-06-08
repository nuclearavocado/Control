"""
    Solve the CartPole environment with LQR and plot it with matplotlib.
"""

import numpy as np
from scipy.integrate import solve_ivp
# Local
from systems.cartpole import CartPole
from algorithms.LQR import lqr
from plot_cartpole import animate

if __name__ == "__main__":
    """
        Define physical system
    """
    # Parameters
    m = 1
    M = 5
    L = 2
    g = -9.81
    d = 1

    s = 1 # pendulum up (s = 1)
    # A: state matrix
    A = np.array([[ 0,  1,          0,                  0],
                  [ 0, -d/M,       -m*g/M,              0],
                  [ 0,  0,          0,                  1],
                  [ 0, -s*d/(M*L), -s*(m + M)*g/(M*L),  0]])
    # B: input matrix
    B = np.array([[ 0         ],
                  [ 1/M       ],
                  [ 0         ],
                  [ s*1/(M*L)]])

    """
        Solve with LQR
    """
    # Q: state cost matrix
    Q = np.array([[ 1,  0,  0,   0],
                  [ 0,  1,  0,   0],
                  [ 0,  0,  10,  0],
                  [ 0,  0,  0,   100]])
    # R: input cost matrix
    R = np.array([[.001]])

    """
        Solve LQR for random initial states
    """

    # Solve for K control matrix with LQR
    K = lqr(A, B, Q, R)
    K = K.reshape((4,)) # want this shape for `control_input`

    """
        Setup cartpole environment
    """
    ref = np.array([0, 0, np.pi, 0]) # reference goal state vector
    ref[0] += np.random.uniform(low=-4, high=4) # random goal state
    def control_input(x):
        # Define our control input function for the CartPole environment
        return sum(-K*(x - ref))
    u = control_input # shorthand
    S = CartPole(m, M, L, g, d, u)

    """
        Solve ODE
    """
    T = 100 # number of timesteps to solve for
    t_span = np.arange(0, T, .1) # timesteps at which to evaluate the solution
    y_0 = [0, 0, np.pi, 0] # default initial conditions
    # Random perturbations
    y_0[0] += np.random.uniform(low=-2.5, high=2.5) # initial x-axis location
    y_0[2] += np.random.uniform(low=-np.pi/4, high=np.pi/4) # initial pole angle
    # Precompute evolution of the state of the system before plotting
    sol = solve_ivp(S.update, [0, T], y_0, t_eval=t_span)

    """
        Plotting
    """
    try:
        # Results from ODESolve
        goal = ref[0]
        x = sol.y[0]
        theta = sol.y[2]
        animate(goal, x, theta, T, M, L)
    except KeyboardInterrupt:
        exit()
