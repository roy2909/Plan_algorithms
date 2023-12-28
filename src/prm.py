import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import networkx as nx

# Sample random points in the configuration space
def sample_points(num_points, space_limits):
    """Generate random points within the given space limits."""
    return np.random.rand(num_points, len(space_limits)) * space_limits

circles = []

def create_random_obstacle(num_obstacles):
    """Create random circular obstacles and store their positions and radii."""
    global circles
    while num_obstacles > 0:
        center_x = np.random.randint(1, 100)
        center_y = np.random.randint(1, 100)
        center = [center_x, center_y]
        radius = np.random.randint(1, 10)
        circles.append([center, radius])
        num_obstacles -= 1

def is_feasible(p1, p2, obstacles):
    """Check if a path between two points is feasible, avoiding obstacles."""
    for obstacle in obstacles:
        center, radius = obstacle
        obstacle_center = np.array(center)

        # Check if the line segment between p1 and p2 intersects with the obstacle
        v1 = p2 - p1
        v2 = obstacle_center - p1
        proj = np.dot(v2, v1) / np.dot(v1, v1)
        closest = p1 + proj * v1
        
        # If the closest point to the obstacle center on the line segment is within the obstacle radius, it's infeasible
        if np.linalg.norm(closest - obstacle_center) < radius:
            return False

    return True


def build_roadmap(points, k_nearest, obstacles):
    """Build a roadmap connecting sampled points based on feasibility."""
    # Create an empty graph
    G = nx.Graph()

    # Add nodes to the graph
    for i, p1 in enumerate(points):
        G.add_node(i, pos=(p1[0], p1[1]))

    # Add edges to the graph based on feasibility
    for i, p1 in enumerate(points):
        distances = np.linalg.norm(points - p1, axis=1)
        nearest_indices = np.argsort(distances)[1:k_nearest+1]
        for j in nearest_indices:
            if is_feasible(p1, points[j], obstacles):
                G.add_edge(i, j)

    return G

def visualize_roadmap_with_path(graph, obstacles, path):
    """Visualize the roadmap with obstacles and highlighted path."""
    plt.figure(figsize=(8, 6))
    plt.title('Probability Road Map with Obstacles')

    pos = nx.get_node_attributes(graph, 'pos')
    nx.draw(graph, pos, with_labels=True, node_color='yellow')

    for obstacle in obstacles:
        center, radius = obstacle
        circle = Circle(center, radius, color='red', fill=True)
        plt.gca().add_patch(circle)

    # Highlight the path on the graph
    edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
    nx.draw_networkx_edges(graph, pos, edgelist=edges, edge_color='orange', width=5.0)

    plt.show()

# Define the configuration space limits
space_limits = np.array([100, 100])
num_of_obstacles = 10

def find_path(start, goal, graph):
    """Find a path from start to goal using Dijkstra's algorithm."""
    path = nx.dijkstra_path(graph, start, goal)
    return path

create_random_obstacle(num_of_obstacles)

num_points = 50
points = sample_points(num_points, space_limits)

k_nearest = 5
graph = build_roadmap(points, k_nearest, circles)
start_point = np.random.randint(len(points))
goal_point = np.random.randint(len(points))

# Find a path from start to goal
path = find_path(start_point, goal_point, graph)
print("Path from", start_point, "to", goal_point, ":", path)

# Visualize the generated roadmap with obstacles
visualize_roadmap_with_path(graph, circles, path)
# Path from 20 to 16 : [20, 42, 22, 45, 12, 39, 36, 35, 46, 30, 34, 21, 16]
# Path from 47 to 24 : [47, 38, 27, 17, 7, 43, 24]