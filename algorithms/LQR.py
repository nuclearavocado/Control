"""
    Adapted from Mark Wilfried Mueller:
    [1] https://www.mwm.im/lqr-controllers-with-python/
    [2] https://github.com/markwmuller/controlpy
"""

import scipy.linalg as LA

def lqr(A, B, Q, R):
    """
        Solve for the LQR controller for a continuous time system.
        A and B are matrices, describing the system dynamics:
            dx/dt = A x + B u
        The controller minimizes the infinite horizon quadratic cost function:
            J = integral (x.T*Q*x + u.T*R*u) dt
            where: Q is a positive semidefinite matrix, and
                   R is a positive definite matrix.

        Ref. Dynamic Programming and Optimal Control, Bertsekas, p.151

        # Arguments:
            A (np.array): state matrix
            B (np.array): input matrix
            Q (np.array): state cost matrix
            R (np.array): input cost matrix
        # Returns
            K (np.array): The optimal feedback control matrix that defines the
                control input u = -Kx based on the cost Q and R.
    """
    # Try to solve the Ricatti equation
    X = LA.solve_continuous_are(A, B, Q, R)
    # Compute the LQR gain
    K = LA.inv(R)@(B.T@X)
    return K
