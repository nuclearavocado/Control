"""
    Plot CartPole environment with matplotlib.
"""

from numpy import sin, cos, sqrt
import matplotlib.pyplot as plt
import matplotlib.lines as mlines # For drawing lines
import matplotlib.patches as mpatches # For drawing rectangles, circles etc.
from matplotlib import animation # For animating
from matplotlib import interactive

def draw_cartpole(ax, goal, x, theta, M, L):
    """
        Animation function to plot the CartPole system that is passed to
        `matplotlib.animation.FuncAnimation`
        # Arguments:
            ax (matplotlib.axes instance): axis to plot to.
            goal (int): target location for the CartPole to reach.
            x (int): x-axis position of the cart.
            theta (int): angle of the pole.
            M (int): mass of the cart.
            L (int): length of the pole.
        # Returns:
            artists (list): list of shapes etc. to plot
    """
    # Empty list of artists (shapes, etc.) to return for matplotlib
    artists = []

    # Locations of the cart and the pole
    cart_x, cart_y = x, 0
    pole_x, pole_y = cart_x + L*sin(theta), cart_y - L*cos(theta)

    # Dimensions
    W = 0.4*sqrt(M) # cart width
    H = 0.25*sqrt(M) # cart height
    # Positions
    y = 0 # cart vertical position
    pole_x = x + L*sin(theta)
    pole_y = y - L*cos(theta)

    # Shapes
    # Draw goal
    artists.append(ax.add_patch(mpatches.Circle([goal, 0], .08, facecolor=[1, 0, 0], zorder=0)))
    # Draw cart
    artists.append(ax.add_patch(mpatches.Rectangle([cart_x - W/2, cart_y - H/2], W, H, facecolor=[0, 0, 0], edgecolor='k', linewidth=1.5)))
    # Draw pole
    artists.append(ax.add_line(mlines.Line2D([cart_x, pole_x], [y + (2*(H/2))/3, pole_y], color=[.8,.6,.4], lw=6, zorder=1)))
    # Draw pole joint/axis
    artists.append(ax.add_patch(mpatches.Circle([cart_x, cart_y + (2*(H/2))/3], .08, facecolor=[.5, .5, .8], zorder=2)))

    return artists

def animate(goal, x, theta, T, M, L):
    # Create new figure with white background
    fig = plt.figure(figsize=(8, 5), facecolor='w')
    # Add a subplot with no frame
    ax = plt.subplot(111, frameon=False)
    # Plot params
    plt.axis("off")
    plt.axis('equal')
    ax.set_xlim(-5, 5)
    ax.set_ylim(-2.5, 2.5)
    # This will be constant, so doesn't need animating
    plt.plot([-10, 10], [0, 0], color='k', linewidth=1) # Draw ground
    # Animation
    FPS = 24
    # Append shapes to a list for ArtistAnimation function
    artists_list = []
    for t in range(0, T):
        artists_list.append(draw_cartpole(ax, goal, x[t], theta[t], M, L))
    # Exits the animation if window closed
    try:
        anim = animation.ArtistAnimation(fig, artists_list, interval=(1/FPS)*1e3,
                                                            blit=True,
                                                            repeat=False)
        plt.show(block=False)
        plt.pause(3.5)
        plt.close()
    except:
        exit()