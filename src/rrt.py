import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Function to check if a point is inside an obstacle
def is_inside_obstacle(point, obstacles):
    for obstacle in obstacles:
        center, radius = obstacle
        if np.linalg.norm(point - center) <= radius:
            return True
    return False

# Function to generate random points within the space limits
def random_point(space_limits):
    return np.random.rand(len(space_limits)) * space_limits

# Function to find the nearest node in the tree to a randomly sampled point
def nearest_node(nodes, point):
    distances = np.linalg.norm(nodes - point, axis=1)
    return np.argmin(distances)

# Function to extend the tree towards a sampled point
def extend_tree(nodes, obstacles, goal, max_distance):
    rand_point = random_point(space_limits)
    nearest_index = nearest_node(nodes, rand_point)
    nearest_node_pos = nodes[nearest_index]

    direction = rand_point - nearest_node_pos
    distance = np.linalg.norm(direction)
    direction = (direction / distance) * min(distance, max_distance)

    new_node = nearest_node_pos + direction

    if not is_inside_obstacle(new_node, obstacles):
        nodes = np.vstack([nodes, new_node])
        return nodes, nearest_index

    return None, None

# Function to generate RRT
def generate_rrt(start, goal, space_limits, obstacles, max_iterations=1000, max_distance=5.0):
    nodes = np.array([start])
    parent_indices = [-1]  # Array to store the index of the parent node for each node

    for _ in range(max_iterations):
        new_nodes, parent_index = extend_tree(nodes, obstacles, goal, max_distance)
        if new_nodes is not None:
            nodes = new_nodes
            parent_indices.append(parent_index)

            if np.linalg.norm(nodes[-1] - goal) < max_distance:
                final_node, final_parent_index = extend_tree(nodes, obstacles, goal, max_distance)
                if final_node is not None:
                    nodes = np.vstack([nodes, final_node])
                    parent_indices.append(final_parent_index)
                    break

    return nodes, parent_indices

# Function to backtrack the path from the goal node to the start node
def backtrack_path(parent_indices):
    path = [len(parent_indices) - 1]
    while path[-1] != 0:
        path.append(parent_indices[path[-1]])
    return path[::-1]

# Function to visualize RRT, obstacles, and the path
def visualize_rrt_with_path(nodes, obstacles, start, goal, path):
    plt.figure(figsize=(8, 6))
    plt.scatter(nodes[:, 0], nodes[:, 1], color='blue', label='RRT Nodes')

    for obstacle in obstacles:
        center, radius = obstacle
        circle = Circle(center, radius, color='red', fill=True)
        plt.gca().add_patch(circle)

    plt.scatter(start[0], start[1], color='green', marker='o', label='Start')
    plt.scatter(goal[0], goal[1], color='purple', marker='o', label='Goal')

    # Plot edges (connections between nodes)
    for i in range(len(nodes)):
        if i != 0:
            plt.plot([nodes[i][0], nodes[parent_indices[i]][0]],
                     [nodes[i][1], nodes[parent_indices[i]][1]], color='black', alpha=0.3)

    # Plot the path
    path_points = nodes[path]
    plt.plot(path_points[:, 0], path_points[:, 1], color='orange', linewidth=2, label='Path')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Rapidly-exploring Random Tree (RRT) with Obstacles')
    plt.legend()
    plt.grid(True)
    plt.show()

# Define the configuration space limits and obstacles
space_limits = [100, 100]
obstacles = [([30, 30], 10), ([70, 70], 7), ([50, 20], 12)]

# Define start and goal points
start_point = np.array([10, 10])
goal_point = np.array([90, 90])

# Generate RRT and find the path
rrt_nodes, parent_indices = generate_rrt(start_point, goal_point, space_limits, obstacles)
path = backtrack_path(parent_indices)

# Visualize RRT, obstacles, and the path
visualize_rrt_with_path(rrt_nodes, obstacles, start_point, goal_point, path)

# Visualize RRT, obstacles, and the path
# visualize_rrt_with_path(rrt_nodes, obstacles, start_point, goal_point, path)
