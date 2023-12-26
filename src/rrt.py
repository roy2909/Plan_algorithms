import matplotlib.pyplot as plt
import numpy as np
import math

class RRT:
    def __init__(self, vertices, obstacles_count):
        self.vertices = vertices
        self.delta = 1
        self.domain = 100
        self.circles = []
        self.obstacles = obstacles_count
        self.nodes = []
        self.parents = [[]]

    def configuration(self):
        return [np.random.rand() * self.domain, np.random.rand() * self.domain]

    def nearest_node(self):
        distances = [math.dist(i, self.q_random) for i in self.nodes]
        self.q_near = self.nodes[distances.index(min(distances))]
        return self.q_near

    def new_configuration(self):
        dist = math.dist(self.q_random, self.q_near)
        x_delta = self.q_random[0] - self.q_near[0]
        y_delta = self.q_random[1] - self.q_near[1]
        self.q_new = [self.q_near[0] + (self.delta / dist) * x_delta, self.q_near[1] + (self.delta / dist) * y_delta]
        return self.q_new

    def generate_obstacles(self):
        while self.obstacles != 0:
            x = np.random.randint(1, 100)
            y = np.random.randint(1, 100)
            radius = np.random.randint(1, 8)
            self.circles.append([[x, y], radius])
            self.obstacles -= 1

    def check_collision(self, vertex):
        for circle in self.circles:
            if math.dist(circle[0], vertex) <= circle[1] or self.path_circle_collision(circle, self.q_near, vertex):
                return True
        return False

    def path_circle_collision(self, circle, point1, point2):
        x_delta = point2[0] - point1[0]
        y_delta = point2[1] - point1[1]
        u = self.calculate_u(circle[0], point1, point2)
        if u < 0:
            closest_point = point1
        elif u > 1:
            closest_point = point2
        else:
            closest_point = (point1[0] + u * x_delta, point1[1] + u * y_delta)
        
        distance = math.dist(closest_point, circle[0])
        if distance <= circle[1]:
            return True
        return False

    def calculate_u(self, point1, point2, point3):
        x_delta = point3[0] - point2[0]
        y_delta = point3[1] - point2[1]
        return ((point1[0] - point2[0]) * x_delta + (point1[1] - point2[1]) * y_delta) / (x_delta * x_delta + y_delta * y_delta)

    def check_collision_free_path(self):
        for circle in self.circles:
            if self.path_circle_collision(circle, self.q_new, self.q_goal):
                return False
        return True

    def visualize(self):
        f, ax = plt.subplots()
        plt.ion()
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_title("RRT with Obstacles")
        self.generate_obstacles()
        self.q_initial = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while math.dist(circle[0], self.q_initial) <= circle[1]:
                self.q_initial = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        self.nodes.append(self.q_initial)
        self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while math.dist(circle[0], self.q_goal) <= circle[1]:
                self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        ax.plot(self.q_initial[0], self.q_initial[1], 'o', color='green')
        ax.plot(self.q_goal[0], self.q_goal[1], 'o', color='orange')
        for circle in self.circles:
            circle = plt.Circle(circle[0], circle[1], color='black', fill=True)
            ax.add_artist(circle)
        for vertex in range(self.vertices):
            self.q_random = self.configuration()
            self.q_near = self.nearest_node()
            self.q_new = self.new_configuration()
            if not self.check_collision(self.q_new):
                self.nodes.append(self.q_new)
                self.parents.append(self.q_near)
                x1 = [self.q_near[0], self.q_new[0]]
                y1 = [self.q_near[1], self.q_new[1]]
                ax.plot(x1, y1, color='blue')
                plt.pause(0.0001)
                if self.check_collision_free_path():
                    x2 = [self.q_new[0], self.q_goal[0]]
                    y2 = [self.q_new[1], self.q_goal[1]]
                    ax.plot(x2, y2, color='blue')
                    plt.pause(0.0001)
                    break
        self.nodes.append(self.q_goal)
        self.parents.append(self.q_new)
        current = self.nodes[-1]
        previous = self.parents[-1]
        while True:
            if previous == []:
                break
            x3 = [current[0], previous[0]]
            y3 = [current[1], previous[1]]
            ax.plot(x3, y3, color='red')
            plt.pause(0.00001)
            current = previous
            if current in self.nodes:
                index = self.nodes.index(current)
            previous = self.parents[index]
        plt.show()
        plt.pause(5)

def main():
    rrt = RRT(2000, 40)
    rrt.visualize()

if __name__ == "__main__":
    main()
