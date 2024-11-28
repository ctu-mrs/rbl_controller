import pickle
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import OccupancyGrid

from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import pdist
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib
import json

# Set global font and plot parameters
fontsize = 22
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.family'] = 'serif'
plt.rcParams.update({'pdf.fonttype': 42})


def plot_hector_map(bag_file_path, topic_name="/uav33/hector_mapping/map", target_time=None):
    # Open the bag file
    bag = rosbag.Bag(bag_file_path, "r")
    
    # Initialize variables for map data
    map_data = None
    width, height = None, None
    resolution = None
    origin_x, origin_y = None, None
    closest_msg_time = None

    # Parse target time into ROS time if given as a float
    if isinstance(target_time, float) or isinstance(target_time, int):
        target_time = rospy.Time.from_sec(target_time)

    # Extract map data from the bag file, finding the closest message to the target time
    for topic, msg, msg_time in bag.read_messages(topics=[topic_name]):
        if target_time is not None:
            # Check if the message time is the closest to the target time
            if closest_msg_time is None or abs(msg_time - target_time) < abs(closest_msg_time - target_time):
                closest_msg_time = msg_time
                width = msg.info.width
                height = msg.info.height
                resolution = msg.info.resolution
                origin_x = msg.info.origin.position.x
                origin_y = msg.info.origin.position.y
                map_data = np.array(msg.data).reshape((height, width))
        else:
            # No target time specified, just take the first message
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            map_data = np.array(msg.data).reshape((height, width))
            break  # Stop after the first message if no target_time is specified

    # Close the bag file
    bag.close()

    # Check if map data was found
    if map_data is None:
        print(f"No map data found in topic {topic_name} around time {target_time.to_sec() if target_time else 'start'}")
        return

    # Convert occupancy grid data to float and handle unknowns
    map_data = np.where(map_data == -1, 0.5, map_data / 100.0)  # Scale to [0, 1]

    # Calculate the extent based on origin and resolution
    extent = [
        origin_x,                          # Left edge in meters
        origin_x + width * resolution,     # Right edge in meters
        origin_y,                          # Bottom edge in meters
        origin_y + height * resolution     # Top edge in meters
    ]

    # Plot the map
    fig, ax = plt.subplots()
    plt.imshow(map_data, cmap="gray_r", origin="lower", extent=extent, zorder=10)
    plt.xlabel("X Position (meters)", fontsize=fontsize)
    plt.ylabel("Y Position (meters)", fontsize=fontsize)
    plt.colorbar(label="Occupancy Probability")


def plot_uav_data():
    # Load data
    data = np.load('uav_data.npz')
    id1_x = data['id1_x']
    id1_y = data['id1_y']
    id3_x = data['id3_x']
    id3_y = data['id3_y']
    id4_x = data['id4_x']
    id4_y = data['id4_y']
    final_x = data['final_x']
    final_y = data['final_y']

    # Load colors
    with open('colors.json', 'r') as json_file:
        color_data = json.load(json_file)

    colors1 = color_data['colors1']
    colors3 = color_data['colors3']
    colors4 = color_data['colors4']
    final_colors = color_data['final_colors']

    # Overlay UAV data
    plt.plot(id1_x, id1_y, color='red', linewidth=1, zorder=5, label="ID 1 Path")
    plt.scatter(id1_x, id1_y, c=colors1, marker='o', s=100, zorder=6)

    plt.plot(id3_x, id3_y, color='blue', linewidth=1, zorder=5, label="ID 3 Path")
    plt.scatter(id3_x, id3_y, c=colors3, marker='o', s=100, zorder=6)

    plt.plot(id4_x, id4_y, color='orange', linewidth=1, zorder=5, label="ID 4 Path")
    plt.scatter(id4_x, id4_y, c=colors4, marker='o', s=100, zorder=6)

    plt.plot(final_x, final_y, color='green', linewidth=2, zorder=5, label="Final Path")
    plt.scatter(final_x, final_y, c=final_colors, marker='o', s=100, zorder=6)

    # Add labels, legend, and grid
    plt.xlabel('x (m)', fontsize=fontsize)
    plt.ylabel('y (m)', fontsize=fontsize)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Save the figure to a PDF
    plt.savefig("hectorplot.pdf")
    plt.show()


# Use the functions with your ROS bag file and UAV data
plot_hector_map("_2024-10-24-17-56-24.bag", target_time=1729785592.149783)  # Replace with your bag file and target time
plot_uav_data()
