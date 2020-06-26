"""
    Solve the CartPole environment with LQR and plot it with matplotlib.
"""

import numpy as np
# Local
from systems.cartpole import CartPole
from algorithms.LQR import lqr
from plot_cartpole import animate

if __name__ == "__main__":
    """
        Define physical system
    """
    # Parameters
    g = 9.81
    m_p, m_c, l, d = 1, 5, 2, 0

    s = 1 # pendulum up (s = 1)
    # A: state matrix
    A = np.array([[ 0,  1,            0,                       0],
                  [ 0, -d/m_c,       m_p*g/m_c,                0],
                  [ 0,  0,            0,                       1],
                  [ 0, -s*d/(m_c*l), s*(m_p + m_c)*g/(m_c*l),  0]])
    # B: input matrix
    B = np.array([[ 0          ],
                  [ 1/m_c      ],
                  [ 0          ],
                  [ s*1/(m_c*l)]])

    """
        Solve with LQR
    """
    # Q: state cost matrix
    Q = np.array([[ 10,  0,  0,   0  ],
                  [ 0,  1,  0,    0  ],
                  [ 0,  0,  100,  0  ],
                  [ 0,  0,  0,   1000]])
    # R: input cost matrix
    R = np.array([[.001]])

    # Solve for K control matrix with LQR
    K = lqr(A, B, Q, R)

    ref = np.array([0, 0, np.pi, 0]) # reference goal state vector
    ref[0] += np.random.uniform(low=-4, high=4) # random goal state
    # Define our control input function for the CartPole environment
    EPS = np.pi/4
    def u(t, x):
        s, c = np.sin(x[2]), np.cos(x[2])
        E = (1/2)*m_p*l**2*x[3]**2 - m_p*g*l*c
        if x[2] > np.pi - EPS and x[2] < np.pi + EPS:
            return sum(-K@(x - ref)) # sum ensures we return a scalar, not an array
        else:
            E_d = m_p*g*l
            E_tilde = E - E_d
            k_E, k_p, k_d = 2, 5, 1
            return k_E*x[3]*c*E_tilde - k_p*x[0] - k_d*x[1]

    """
        Setup cartpole environment
    """
    T = 200 # number of timesteps to simulate
    # Define initial conditions
    x_0 = [0, 0, 0*np.pi, 0]
    # Add some random perturbations
    x_0[0] += np.random.uniform(low=-2.5, high=2.5) # initial x-axis location
    x_0[2] += np.random.uniform(low=-np.pi/4, high=np.pi/4) # initial pole angle
    S = CartPole(u, T, x_0=x_0)
    X = S.simulate()

    """
        Plot
    """
    try:
        # Results from ODESolve
        goal = ref[0]
        x = X[0]
        theta = X[2]
        animate(goal, x, theta, T, S.m_c, S.l)
    except KeyboardInterrupt:
        exit()
