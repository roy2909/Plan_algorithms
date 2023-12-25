from turtle import distance
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math

class RRT:
    def __init__(self,vertices,obstacles_count):
        self.q_initial=[]
        self.q_goal=[]
        self.vertices=vertices
        self.delta=1
        self.domain=100
        self.circles=[]
        self.obstacles=obstacles_count
        self.list=[]
        self.prev_list=[[[]]]

    def configuration(self):
        self.q_random=[np.random.rand()*self.domain,np.random.rand()*self.domain]
        return self.q_random
    
    def nearest_vertex(self):
        distance=[]
        for i in self.list:
            dist=math.dist(i,self.q_random)
            distance.append(dist)
        self.q_near=self.list[distance.index(min(distance))]
        return self.q_near
    
    def update_configuration(self):
        dist=math.dist(self.q_random,self.q_near)
        x_del = self.q_random[0] - self.q_near[0]
        y_del = self.q_random[1] - self.q_near[1]
        q_new_x = self.q_near[0] + (self.delta / dist) * x_del
        q_new_y = self.q_near[1] + (self.delta / dist) * y_del
        self.q_new = [q_new_x, q_new_y]
        return self.q_new
    
    def circular_obstacles(self):
        while self.obstacles!=0:
            x=np.random.randint(1,100)
            y=np.random.randint(1,100)
            radius=np.random.randint(1,8)
            center=[x,y]
            self.circles.append([center,radius])
            self.obstacles-=1

    def u_check(self, point1, point2, point3):
        """Calculate u required to check for path collision"""
        x_delta = point3[0] - point2[0]
        y_delta = point3[1] - point2[1]
        u = ((point1[0] - point2[0]) * x_delta + (point1[1] - point2[1]) * y_delta) / (x_delta * x_delta + y_delta * y_delta)
        return u
    
    def vertex_circle_collision(self,vertex,circle):
        if math.dist(circle[0], vertex) <= circle[1]:
            return True
        return False
    
    def path_collision(self, circle, point1, point2):
        """Check if the path from a vertex to another vertex intersects with a circle"""
        x_delta = point2[0] - point1[0]
        y_delta = point2[1] - point1[1]
        u = self.u_check(circle[0], point1, point2)
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
    def check_collision(self, vertex):
        """Check if there is any collision"""
        for circle in self.circles:
            if self.vertex_circle_collision(vertex, circle) or self.path_collision(circle, self.q_near, vertex):
                return True
        return False

    def check_collision_free_path(self):
        """Check for a collision free path from a new vertex to the goal"""
        for circle in self.circles:
            if self.path_collision(circle, self.q_new, self.q_goal):
                return False
        return True

    def generate_random_start(self):
        """Generate a random start location"""
        self.q_initial = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.vertex_circle_collision(self.q_initial, circle):
                self.q_initial = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_initial
    
    def generate_random_goal(self):
        """Generate a random goal location"""
        self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.vertex_circle_collision(self.q_goal, circle):
                self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_goal