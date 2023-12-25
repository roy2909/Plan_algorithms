import matplotlib.pyplot as plt
import random
import math
from matplotlib.collections import LineCollection
import numpy as np
from matplotlib.animation import FuncAnimation

class RRT:
    def __init__(self, domain, q_init, delta, K):
        self.domain = domain
        self.q_init = q_init
        self.delta = delta
        self.K = K
        self.G = [q_init]
        self.line_segments = []

    def nearest_neighbor(self, q_rand):
        return min(self.G, key=lambda q: ((q[0] - q_rand[0]) ** 2 + (q[1] - q_rand[1]) ** 2) ** 0.5)

    def plan(self):
        for _ in range(self.K):
            q_rand = (random.uniform(self.domain[0], self.domain[1]),
                      random.uniform(self.domain[0], self.domain[1]))

            q_near = self.nearest_neighbor(q_rand)

            dist = (math.sqrt((q_rand[0] - q_near[0]) ** 2 + (q_rand[1] - q_near[1]) ** 2))
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
        X_values = [q[0] for q in self.G[:i]]
        Y_values = [q[1] for q in self.G[:i]]
        line_segments = self.line_segments[:i]
        collection.set_segments(line_segments)
        scatter.set_offsets(np.column_stack((X_values, Y_values)))
        return scatter, collection

    def animate(self):
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
