import numpy as np
from numpy import sin, cos

class CartPole:
    """
        Description:
            A pole is attached by an un-actuated joint to a cart, which moves
            along a track. The cart is set to random initial conditions, and
            must reach a random target location while keeping the pole vertical.
            The environment is designed to look like the OpenAI gym environment,
            but is more challenging than that implemented by Rich Sutton et al.
            and used in OpenAI gym.
            The CartPole must simultaneously move to a target location and keep
            the pole balanced. It obeys the Lagrangian dynamics formulation,
            includes friction, and is given random initial conditions in a much
            more extreme range.
    """
    def __init__(self, m, M, L, g, d, u):
        """
            # Arguments:
                m (int): mass of the pole.
                M (int): mass of the cart.
                L (int): length of the pole.
                g (int): gravity (-9.81).
                d (int): track friction.
                u (func): function defining how the control input is calculated.
        """
        self.params = (m, M, L, g, d)
        self.dx = np.zeros((4,))
        self.control_input = u

    def update(self, t, x):
        """
            Updates the Lagrangian formulation of the CartPole system.
            # Arguments:
                t (int): current time.
                x (int): state of the system
                    x[0] = x-axis position of the cart
                    x[1] = x-axis velocity of the cart
                    x[2] = angle of the pole
                    x[3] = angular velocity of the pole
            # Returns:
                dx (np.array): change in state of the system (follows same
                    indexing as above).
        """
        m, M, L, g, d = self.params
        u = self.control_input(x)
        s = sin(x[2])
        c = cos(x[2])
        D = m*L**2*(M + m*(1 - c**2))
        self.dx[0] = x[1]
        self.dx[1] = (1/D)*(-m**2*L**2*g*c*s + m*L**2*(m*L*x[3]**2*s - d*x[1])) + m*L**2*(1/D)*u
        self.dx[2] = x[3]
        self.dx[3] = (1/D)*((m + M)*m*g*L*s - m*L*c*(m*L*x[3]**2*s - d*x[1])) - m*L*c*(1/D)*u
        return self.dx
