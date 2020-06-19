import numpy as np
from scipy.integrate import solve_ivp

class CartPole:
    """
        Description:
            A pole is attached by an un-actuated joint to a cart, which moves
            along a track. The cart is set to random initial conditions, and
            must reach a random target location while keeping the pole vertical.
            The environment is designed to look like the OpenAI gym environment,
            but is more challenging than that implemented by Rich Sutton et al.
            and used in OpenAI gym.
            The cart-pole must simultaneously move to a target location and keep
            the pole balanced. It obeys the Lagrangian dynamics formulation
            and is given random initial conditions in a much more extreme range.
    """
    def __init__(self, u, T, m_p=1,
                             m_c=5,
                             l=2,
                             x_0=[0, 0, np.pi, 0],
                             g=9.81,
                             manipulator_eqns=False):
        """
            # Arguments:
                u (func): function defining the control input.
                T (int): length of time for which to simulate the cart-pole.
                m_p (int): mass of the pole.
                m_c (int): mass of the cart.
                l (int): length of the pole.
                x_0 (numpy.array): default initial conditions
                    x_0[0] = initial x-axis position of the cart, `x`.
                    x_0[1] = initial x-axis velocity of the cart, `x_dot`.
                    x_0[2] = initial angle of the pole, `theta`.
                    x_0[3] = initial angular velocity of the pole, `theta_dot`.
                g (int): scalar force due to gravity.
                manipulator_eqns (bool): choose whether to solve the environment
                    by inverting the manipulator equations, or by solving
                    directly for the accelerations. The math is different, but
                    the outcome is the same.
        """
        # Arguments
        self.manipulator_eqns = manipulator_eqns
        self.u, self.T = u, T
        self.g = g
        self.m_p, self.m_c = m_p, m_c
        self.l = l
        self.x_0 = x_0

    def update(self, t, x):
        """
            Updates the Lagrangian formulation of the cart-pole system.
            # Arguments:
                t (int): current time.
                x (int): state of the system
                    x[0] = x-axis position of the cart, `x`.
                    x[1] = x-axis velocity of the cart, `x_dot`.
                    x[2] = angle of the pole, `theta`.
                    x[3] = angular velocity of the pole, `theta_dot`.
            # Returns:
                dx (numpy.array): change in state of the system (follows same
                    indexing as above).
        """
        # For readability
        g = self.g
        m_p, m_c = self.m_p, self.m_c
        l = self.l
        s, c = np.sin(x[2]), np.cos(x[2])
        u = self.u(t, x)
        dx = np.zeros((4,))

        if not self.manipulator_eqns:
            den = (m_c + m_p*s**2)
            q_ddot = np.zeros((2,1))
            q_ddot[0] = (1/den)*(u + m_p*s*(l*x[3]**2 + g*c))
            q_ddot[1] = (1/(l*den))*(-u*c - m_p*l*x[3]**2*c*s - (m_c + m_p)*g*s)
        else:
            # Mass matrix
            M = np.array([[ m_c + m_p,  m_p*l*c ],
                          [ m_p*l*c,    m_p*l**2]])
            # Coriolis terms
            C = np.array([[ 0, -m_p*l*x[3]*s],
                          [ 0,  0           ]])
            # Torque
            tau_g = -g*np.array([[ 0      ],
                                 [ m_p*l*s]])
            # B
            B = np.array([[ 1],
                          [ 0]])
            # Calculate angular acceleration
            q_dot = np.array([[x[1]],
                              [x[3]]])
            Cv = np.dot(C, q_dot)
            q_ddot = np.dot(np.linalg.inv(M), (tau_g + np.dot(B, u) - Cv))

        dx[0] = x[1]
        dx[1] = q_ddot[0]
        dx[2] = x[3]
        dx[3] = q_ddot[1]
        return dx

    def simulate(self):
        # Solve ODE
        sol = solve_ivp(self.update, [0, self.T], self.x_0, t_eval=np.arange(0, self.T, .1))
        return sol.y
