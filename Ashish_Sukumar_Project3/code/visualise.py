#!/usr/bin/env python

import matplotlib
import argparse
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import os 
import sys
from matplotlib import patches
import math
import numpy as np

def get_obstacles_from_file(filepath):
### Function to read the obstacles from the file and return the list of obstacles
### Each obstacle is represented as a list of 4 elements: x, y, width, height
### Input: filepath - path to the file containing the obstacles
### Output: List of obstacles
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            obstacles = []
            
            for line in lines:
                obstacle = line.split()
                obstacles.append([float(obstacle[i]) for i in range(len(obstacle))])
        obstacles = list(filter(lambda x: x, obstacles))
        return obstacles
    
    else:
        print("Obstacle file not found - Please provide the right path")
        sys.exit(0)

def get_path_from_file(filepath):
### Function to read the path from the file and return the path
### The path is represented as a list of poses
### The first line of the file contains the configuration space and the size of the robot (if not a point robot)
### Input: filepath - path to the file containing the path
### Output: Configuration space, robot size (if not a point robot), List of poses
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            path = []
            for l in lines:
                configuration = l.split()
                path.append([float(ci) for ci in configuration])

        path = list(filter(lambda x: x, path))
        return path
    else:
        print("Path file not found - Please provide the right path")
        sys.exit(0)

def get_corners(pose, robot_size):
### Function to get the corners of the robot given the pose and the size of the square robot
### Input: pose - (x, y, theta) of the robot, robot_size - size of the robot
### Output: List of corners of the robot for given pose in SE2
    x, y, theta = pose
    corners = [(x + robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta)), \
                (x + robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta))]
    return corners

def set_plot_properties(ax):
### Function to set the properties of the plot
    ax.grid(True)
    #adjustable, box
    ax.set_aspect('equal', 'box')
    ax.set_facecolor('#f0f0f0')
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_title('Environment and Path Visualization')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')

def forward_kinematics(config, start_point, start_angle):
    """Compute the joint coordinates given a configuration of joint angles.
    The last endpoint would be used for visualization of the sampling
    arguments:
        config: A list of joint angles in radians.

    return:
        edges: A list of joint coordinates.
    """
    # Initialize the starting point as the fixed base
    num_joints = len(config)
    link_length = 1

    joint_positions = [start_point]
    start_point = np.array(start_point)
    angle = start_angle 

    # Compute the end points of each joint based on the configuration
    for i in range(num_joints):
        # Compute the end point of the current link
        angle += config[i]
        end_point = start_point + np.array(
            [link_length * np.cos(angle), link_length * np.sin(angle)])
        # Add the endpoint and update the start point for the next joint
        joint_positions.append(end_point.tolist())
        start_point = end_point

    return joint_positions

def draw_environment(ax, obstacles):
    for obstacle in obstacles:
        ax.add_patch(patches.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], fill=True, color='black'))


def plot_environment_and_path(obstacles, path):
    _, ax = plt.subplots()
    draw_environment(ax, obstacles)

    #plot chain
    for config in path:
       corners = get_corners(config[:3], 1)
       polygon = patches.Polygon(corners, fill=False, color='green')
       ax.add_patch(polygon)

       positions = forward_kinematics(config[3:], config[:2], config[2])
       # Draw lines between each joint
       for i in range(len(positions) - 1):
           line = np.array([positions[i], positions[i + 1]])
           ax.plot(line[:, 0], line[:, 1], color="g")
       # Draw joint
       for i in range(len(positions)):
           ax.scatter(positions[i][0], positions[i][1], s=2, c="green")

    #TODO reduce this. 
    set_plot_properties(ax)
    plt.savefig('path_visualization.png', dpi=300)
    plt.close()

def animate_environment_and_path(obstacles, path):

    fig, ax = plt.subplots()

    draw_environment(ax, obstacles)
    line, = ax.plot([], [], 'g-', linewidth=1, label='Path')
    point, = ax.plot([], [], 'go', markersize=2, label='Pose')
    corners = get_corners(path[0][:3], 1)
    polygon = patches.Polygon(corners, fill=False, color='green', label='Robot')
    ax.add_patch(polygon)
    set_plot_properties(ax)

    def update(frame):
        config = path[frame] 
        x_coords = [pose[0] for pose in  forward_kinematics(config[3:], config[:2], config[2])]
        y_coords = [pose[1] for pose in  forward_kinematics(config[3:], config[:2], config[2])]
        line.set_data(x_coords, y_coords)
        point.set_data(x_coords, y_coords)
        corners = get_corners(config[:3], 1)
        polygon.set_xy(corners)
        return line, point, polygon

    ani = FuncAnimation(fig, update, frames=len(path), blit=True, repeat=False)
    ani.save('path_visualization.gif', writer='imagemagick', fps=30)
    plt.close()

    
def main():
    
    parser = argparse.ArgumentParser(description='Visualize the path of a robot in an environment with obstacles.')
    parser.add_argument('--environment', type=str, default='environemnt', help='Name of the obstacles file')
    parser.add_argument('--path', type=str, default='path', help='Name of the path file') 
    parser.add_argument('--num_links', type=str, default='5', help='Number of Links on the Chain') 
    args = parser.parse_args()

    print("***" * 19 + f"\n Visualising the environment {args.environment} and path {args.path}\n" + "***" * 19)
    print("Instructions: \n1. Please ensure that the obstacles text file should contain the obstacle data in the format: x y width height")
    print("2. The path text file should specify the configuration space and the size of the robot (if not a point robot) on the first line and the poses in subsequent lines \n")

    #Get absolute path of the files
    obs_path = os.path.join(os.getcwd(), args.environment)
    path_path = os.path.join(os.getcwd(), args.path)
    obstacles = get_obstacles_from_file(obs_path)
    path = get_path_from_file(path_path)

    print("Path and Obstacle data loaded successfully")

    plot_environment_and_path(obstacles, path)
    animate_environment_and_path(obstacles, path)

if __name__ == "__main__":
    main()
