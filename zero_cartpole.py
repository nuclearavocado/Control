"""
    Solve the CartPole environment with LQR and plot it with matplotlib.
"""

import numpy as np
# Local
from systems.cartpole import CartPole
from plot_cartpole import animate

if __name__ == "__main__":
    """
        Setup cartpole environment
    """
    def u(t, x):
        # Define our control input function for the CartPole environment
        return 0
    T = 100 # number of timesteps to solve for
    # Define initial conditions
    x_0 = [0, 0, np.pi, 0]
    # Add some random perturbations
    x_0[0] += np.random.uniform(low=-2.5, high=2.5) # initial x-axis location
    x_0[2] += np.random.uniform(low=-np.pi/4, high=np.pi/4) # initial pole angle
    S = CartPole(u, T, x_0=x_0)
    X = S.simulate()

    """
        Plotting
    """
    try:
        # Results from ODESolve
        x = X[0]
        theta = X[2]
        animate(0, x, theta, T, S.m_c, S.l)
    except KeyboardInterrupt:
        exit()
