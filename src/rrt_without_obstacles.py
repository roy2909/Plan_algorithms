import matplotlib.pyplot as plt
import random
import math
from matplotlib.collections import LineCollection
import numpy as np
from matplotlib.animation import FuncAnimation

class RRT:
    """
    Rapidly-Exploring Random Tree (RRT) algorithm for path planning in a 2D space.

    Attributes:
        domain (tuple): Tuple defining the rectangular space (min, max) for X and Y coordinates.
        q_init (tuple): Initial configuration (x, y) in the space.
        delta (float): Step size for extending the tree.
        K (int): Number of iterations for tree expansion.
        G (list): List containing the vertices of the RRT.
        line_segments (list): List of line segments formed by RRT vertices.

    Methods:
        nearest_neighbor(q_rand): Finds the nearest neighbor of a randomly sampled point.
        plan(): Constructs the RRT by iteratively expanding the tree.
        animate_growth(i, scatter, collection): Animation function for visualizing tree growth.
        animate(): Initializes and displays the RRT animation.
    """

    def __init__(self, domain, q_init, delta, K):
        """
        Initializes the RRT with given parameters.

        Args:
            domain (tuple): Tuple defining the rectangular space (min, max) for X and Y coordinates.
            q_init (tuple): Initial configuration (x, y) in the space.
            delta (float): Step size for extending the tree.
            K (int): Number of iterations for tree expansion.
        """
        self.domain = domain
        self.q_init = q_init
        self.delta = delta
        self.K = K
        self.G = [q_init]
        self.line_segments = []

    def nearest_neighbor(self, q_rand):
        """
        Finds the nearest neighbor of a randomly sampled point in the RRT.

        Args:
            q_rand (tuple): Randomly sampled point (x, y).

        Returns:
            tuple: Nearest neighbor to the sampled point.
        """
        return min(self.G, key=lambda q: ((q[0] - q_rand[0]) ** 2 + (q[1] - q_rand[1]) ** 2) ** 0.5)

    def plan(self):
        """
        Constructs the RRT by iteratively expanding the tree based on the given parameters.
        """
        for _ in range(self.K):
            q_rand = (random.uniform(self.domain[0], self.domain[1]),
                      random.uniform(self.domain[0], self.domain[1]))

            q_near = self.nearest_neighbor(q_rand)

            dist = math.sqrt((q_rand[0] - q_near[0]) ** 2 + (q_rand[1] - q_near[1]) ** 2)
            if dist > self.delta:
                q_new = (
                    q_near[0] + (self.delta / dist) * (q_rand[0] - q_near[0]),
                    q_near[1] + (self.delta / dist) * (q_rand[1] - q_near[1])
                )
            else:
                q_new = q_rand
            segments = [(q_near[0], q_near[1]), (q_new[0], q_new[1])]
            self.line_segments.append(segments)
            self.G.append(q_new)

    def animate_growth(self, i, scatter, collection):
        """
        Animation function for visualizing the growth of the RRT.

        Args:
            i (int): Frame index for the animation.
            scatter (matplotlib.collections.PathCollection): Scatter plot for RRT vertices.
            collection (matplotlib.collections.LineCollection): Line collection for RRT edges.

        Returns:
            tuple: Scatter and collection objects for the animation frame.
        """
        X_values = [q[0] for q in self.G[:i]]
        Y_values = [q[1] for q in self.G[:i]]
        line_segments = self.line_segments[:i]
        collection.set_segments(line_segments)
        scatter.set_offsets(np.column_stack((X_values, Y_values)))
        return scatter, collection

    def animate(self):
        """
        Initializes and displays the RRT animation.
        """
        fig, ax = plt.subplots()
        scatter = ax.scatter([], [], s=5, c='r', label='RRT Vertices')
        collection = LineCollection([], colors='black', linewidths=1, alpha=0.5)
        ax.add_collection(collection)
        ax.scatter(self.q_init[0], self.q_init[1], s=50, c='g', marker='o', label='Initial Configuration')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Rapidly-Exploring Random Tree ')
        ax.set_xlim(*self.domain)
        ax.set_ylim(*self.domain)
        ax.legend()
        ax.grid(True)

        ani = FuncAnimation(fig, self.animate_growth, frames=len(self.G) + 1, fargs=(scatter, collection), repeat=False)
        plt.show()

if __name__ == "__main__":
    D = (0, 100)
    X, Y = D, D
    q_init = (50, 50)
    delta = 1
    K = 500

    rrt = RRT(domain=X, q_init=q_init, delta=delta, K=1000)
    rrt.plan()
    rrt.animate()
