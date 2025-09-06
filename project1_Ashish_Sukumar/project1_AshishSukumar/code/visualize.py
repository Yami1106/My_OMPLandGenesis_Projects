import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

def plot_graphml(file_path):
    # Load the GraphML file
    graph = nx.read_graphml(file_path)
    
    # Extract node positions from the GraphML file (assuming they are stored as 'x' and 'y' attributes)
    pos = {}
    for node, data in graph.nodes(data=True):
        x, y = map(float, data['coords'].split(','))
        pos[node] = (x, y)

    # Draw the nodes as small blue circles, without edges
    fig, ax = plt.subplots()
    nx.draw_networkx_nodes(graph, pos, node_size=1, node_color="blue", node_shape='o', ax=ax)
    
    # Draw the nodes as small blue circles (optional but ensures circular shape)
    # Plot the circle with radius 10
    circle = plt.Circle((0, 0), 10, color='blue', fill=False)
    ax.add_patch(circle)

    # Define the obstacle coordinates
    obstacle_coords = [(-3, 0), (-1, -2 ),  (-3, -4 ), (-5, -2)]
    
    # Create and add the obstacle polygon
    obstacle = Polygon(obstacle_coords, closed=True, color='red', alpha=0.3)
    ax.add_patch(obstacle)

    ax.set_aspect('equal')
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    #save the plot as a png image using the name of the graph as a name of the image
    image_name = f"{file_path.split('/')[-1].split('.')[0]}.png"
    plt.savefig(image_name)

# Example usage
plot_graphml('/code/build/naive_samples.graphml')
plot_graphml('/code/build/correct_samples.graphml')
