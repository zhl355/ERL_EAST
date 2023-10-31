#!/usr/bin/env python3
"""
Jackal Data Visualization in Python

---

UCSD ERL Y. Yi, v1.0

---
"""

import os
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable
from matplotlib.legend_handler import HandlerLineCollection
import yaml
import pickle as pkl


class HandlerColorLineCollection(HandlerLineCollection):
    """
    Class override for color bar in legend
    """

    def create_artists(self, legend, artist, xdescent, ydescent,
                       width, height, fontsize, trans):
        x = np.linspace(0, width, self.get_numpoints(legend) + 1)
        y = np.zeros(self.get_numpoints(legend) + 1) + height / 2. - ydescent
        points_ic = np.array([x, y]).T.reshape(-1, 1, 2)
        segments_ic = np.concatenate([points_ic[:-1], points_ic[1:]], axis=1)
        lc_ic = LineCollection(segments_ic, cmap=artist.cmap,
                               transform=trans)
        lc_ic.set_array(x)
        lc_ic.set_linewidth(artist.get_linewidth())
        return [lc_ic]


class DataDecoder:
    def __init__(self):
        """
        Data decoder class, define container for each topic
        """

        # ----------------- ref_gvn_status ---------------------
        # Multi column data
        self.rbt_pose2D = []
        self.gvn_pose2D = []
        self.lpg_pose2D = []

        # Single column data
        self.dQcO = []
        self.dIgO = []
        self.dIcO = []
        self.dIrO = []
        self.deltaE = []
        self.gvn_status = []

        # ----------------- gazebo_p3d_odom ---------------------
        # Multi column data
        self.odom_pose = []
        self.odom_twist = []

        # ----------------- cone_controller_debug ---------------------
        # Single column data
        self.sddm_boost = []
        self.cone_controller_status = []

        # Multi column data
        self.llc_pose = []
        self.cone_controller_timestamp = []

        # ----------------- path ---------------------
        # Multi column data
        self.path = []

    def ref_gvn_status_msg_decoder(self, dataframe_raw):
        """
        Data Decoder for ref_gvn_status msg
        """
        dataframe_var = dataframe_raw["_ref_gvn_status"]
        self.rbt_pose2D = np.column_stack((dataframe_var['rbt_pose2D.x'].to_numpy(),
                                           dataframe_var['rbt_pose2D.y'].to_numpy(),
                                           dataframe_var['rbt_pose2D.theta'].to_numpy()))

        self.gvn_pose2D = np.column_stack((dataframe_var['gvn_pose2D.x'].to_numpy(),
                                           dataframe_var['gvn_pose2D.y'].to_numpy(),
                                           dataframe_var['gvn_pose2D.theta'].to_numpy()))

        self.lpg_pose2D = np.column_stack((dataframe_var['lpg_pose2D.x'].to_numpy(),
                                           dataframe_var['lpg_pose2D.y'].to_numpy(),
                                           dataframe_var['lpg_pose2D.theta'].to_numpy()))

        self.dQcO = dataframe_var['dQcO'].to_numpy()
        self.dIgO = dataframe_var['dIgO'].to_numpy()
        self.dIcO = dataframe_var['dIcO'].to_numpy()
        self.dIrO = dataframe_var['dIrO'].to_numpy()
        self.deltaE = dataframe_var['deltaE'].to_numpy()
        self.gvn_status = dataframe_var['gvn_status'].to_numpy()

    def gazebo_p3d_odom_msg_decoder(self, dataframe_raw):
        """
        Data Decoder for gazebo_p3d_odom msg
        """
        dataframe_var = dataframe_raw["gazebo_p3d_odom"]
        self.odom_pose = np.column_stack((dataframe_var['pose.pose.position.x'].to_numpy(),
                                          dataframe_var['pose.pose.position.y'].to_numpy()))

        self.odom_twist = np.column_stack((dataframe_var['twist.twist.linear.x'].to_numpy(),
                                           dataframe_var['twist.twist.linear.y'].to_numpy()))

    def cone_controller_debug_msg_decoder(self, dataframe_raw):
        """
        Data Decoder for cone_controller_debug
        """
        dataframe_var = dataframe_raw["cone_controller_debug"]
        self.llc_pose = np.column_stack((dataframe_var['z.x'].to_numpy(),
                                         dataframe_var['z.y'].to_numpy(),
                                         dataframe_var['z.theta'].to_numpy()))

        self.cone_controller_timestamp = np.column_stack((dataframe_var['header.stamp.secs'].to_numpy(),
                                                          dataframe_var['header.stamp.nsecs'].to_numpy()))

        self.sddm_boost = dataframe_var['sddm_boost'].to_numpy()
        self.cone_controller_status = dataframe_var['cone_controller_status'].to_numpy()

    def path_msg_decoder(self, dataframe_raw):
        """
        Data Decoder for cone_controller_debug
        """
        dataframe_var = dataframe_raw["_path"]
        series_path = dataframe_var["poses"]

        final_path = series_path.iat[-1]
        final_path = final_path.replace('[', '').replace(']', '').replace(', header:', '\nheader:').replace('header:',
                                                                                                            '---\nheader:')
        dict_path = list(yaml.load_all(final_path, Loader=yaml.SafeLoader))

        waypoint_list = []
        for waypoint in dict_path:
            waypoint_list.append(np.array([waypoint['pose']['position']['x'], waypoint['pose']['position']['y']]))

        waypoint_npa = np.array(waypoint_list)
        self.path = np.flip(waypoint_npa, axis=0)


def jackal_race_world_plotter(ax, pickle_dir):
    """
    Load Jackal_Race Grid Map for Pickal File and plot it as scatter
    """
    res_dict = pkl.load(open(pickle_dir, "rb"), encoding='latin1')

    # map decoding
    map_raw_data = np.array(res_dict['map_data'], dtype=np.int8)
    map_res = res_dict["map_resolution"]
    map_height = res_dict["map_height"]
    map_width = res_dict["map_width"]

    # Convert 2d cell location information into meters
    grid_x_axis = np.linspace(0, map_res * map_width, map_width) - (map_width - 1) / 2 * map_res
    grid_y_axis = np.linspace(0, map_res * map_height, map_height) - (map_height - 1) / 2 * map_res

    # get 2d location information in meters for each grid value
    grid_X, grid_Y = np.meshgrid(grid_x_axis, grid_y_axis)

    # stack together for scatter
    grid_loc_2d_meters = np.column_stack((np.ndarray.flatten(grid_X),
                                          np.ndarray.flatten(grid_Y),
                                          map_raw_data))

    # only legend once
    first_point = True

    for grid in grid_loc_2d_meters:
        # grid value greater 80 indicates obstacle
        if grid[2] >= 80:
            if first_point:
                # again, only legend once
                ax.scatter(grid[0], grid[1], c='grey', marker='s', s=20, label='Obstacles')
                first_point = False
            else:
                ax.scatter(grid[0], grid[1], c='grey', marker='s', s=20)


def load_entire_dir(dir_str, topic_list):
    """
    Load every rosbag in provided dir
    """
    # create dir list
    dir_list = os.listdir(dir_str)

    # empty container
    bag_list = []
    data_bag = {}

    # get path for every file end with .bag
    for item in dir_list:
        if item.endswith('.bag'):
            bag_list.append(item)

    # enumerate each bag
    for bag in bag_list:
        decoded_bag = bagreader(dir_str + "/" + bag)

        # create list based on topics
        data_bag[bag.rsplit(".")[0]] = {}

        # enumerate topics
        for topic in topic_list:
            data_bag[bag.rsplit(".")[0]][topic.rsplit("/")[-2] + "_" + topic.rsplit("/")[-1]] = pd.read_csv(
                decoded_bag.message_by_topic(topic))


def load_single_bag(path_to_bag, topic_list):
    """
    Load a single rosbag
    """
    # empty container
    data_bag = {}

    # read bag
    decoded_bag = bagreader(path_to_bag)

    # create list based on topics
    data_bag[(path_to_bag.rsplit("/")[-1]).rsplit(".")[0]] = {}

    # enumerate topics
    for topic in topic_list:
        data_bag[(path_to_bag.rsplit("/")[-1]).rsplit(".")[0]][
            topic.rsplit("/")[-2] + "_" + topic.rsplit("/")[-1]] = pd.read_csv(decoded_bag.message_by_topic(topic))

    return data_bag


def draw_path2d(ax, waypoints, lc='blue', lw=2, lstr='path', ps='-'):
    """
    draw 2d path consists of waypoints (num_pts, 3)
    """
    ax.plot(waypoints[:, 0], waypoints[:, 1], c=lc, ls=ps, label=lstr, lw=lw)

    return ax


def draw_velocity_arrow(ax, pose_2d,
                        arrow_color='magenta',
                        arrow_scale=0.5, arrow_head_width=0):
    """
    draw velocity arrow inferred from 2d odom information
    """
    # differentiating 2d odom information
    rbt_pose_diff = np.diff(pose_2d[:, 0:2], axis=0)

    # rotate to perpendicular direction to trajectory
    rotation_matrix = np.array([[0, 1],
                                [-1, 0]])
    rbt_pose_diff_perp = (rotation_matrix @ rbt_pose_diff.T).T

    # draw arrow
    ax.quiver(pose_2d[0:-1:2, 0], pose_2d[0:-1:2, 1], rbt_pose_diff_perp[0::2, 0],
              rbt_pose_diff_perp[0::2, 1], color=arrow_color, scale=arrow_scale, headwidth=arrow_head_width)

    return ax


def draw_plot_jackal_race(plot_name, gvn_bag, path_bag, grid_map_pkl):
    """
    Plot python visualization of corresponding simulation
    """

    # Create decoder class
    decoder = DataDecoder()

    # Decode corresponding topic into numpy array
    decoder.gazebo_p3d_odom_msg_decoder(gvn_bag)
    decoder.ref_gvn_status_msg_decoder(gvn_bag)
    decoder.cone_controller_debug_msg_decoder(gvn_bag)

    # Decode path into numpy array
    decoder.path_msg_decoder(path_bag)

    # Calculate duration
    end_time_index = np.argwhere(decoder.cone_controller_status == 2)[0]
    duration_stamp = decoder.cone_controller_timestamp[end_time_index, :] - decoder.cone_controller_timestamp[0, :]
    duration = np.around(duration_stamp[0, 0] + duration_stamp[0, 1] * 1e-9, decimals=2)

    # Differentiating path
    path_diff = np.diff(decoder.path, axis=0)
    path_length = np.around(np.sum(np.sqrt(np.sum(path_diff * path_diff, axis=1))), decimals=2)

    # Obtain start and end points
    start_point = decoder.path[0, :]
    end_point = decoder.path[-1, :]

    # Create fig
    fig_1 = plt.figure(figsize=(8, 6))
    fig_1_ax1 = fig_1.add_subplot(111)

    # Draw Obstacles
    jackal_race_world_plotter(fig_1_ax1, grid_map_pkl)

    # Draw planning path
    fig_1_ax1 = draw_path2d(fig_1_ax1, decoder.path, 'black', 2, 'Planning Path', '--')

    # Draw velocity arrow
    fig_1_ax1 = draw_velocity_arrow(fig_1_ax1, decoder.odom_pose)

    # ----------- Multicolor Traj ------------
    # Divide path into segments
    points = (decoder.odom_pose[:, 0:2]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Determine colormap upper and lower bounds
    norm = plt.Normalize(0, 2)
    lc = LineCollection(segments, cmap='RdYlGn', norm=norm)

    # Draw multicolor path indicating dIrO
    lc.set_array(decoder.dIrO)
    lc.set_linewidth(4)

    # Draw colorbar
    multi_color_line = fig_1_ax1.add_collection(lc)
    divider = make_axes_locatable(fig_1_ax1)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    fig_1.colorbar(multi_color_line, cax=cax)

    # ----------------------------------------

    # Draw start and end points
    fig_1_ax1.scatter(start_point[0], start_point[1], c='red', marker='*', s=320)
    fig_1_ax1.scatter(end_point[0], end_point[1], c='green', marker='*', s=320)

    # Set upper and lower limits for x and y axis
    fig_1_ax1.set_xlim(right=5.0)
    fig_1_ax1.set_ylim(top=2.5)
    fig_1_ax1.set_aspect('equal')

    fig_1_ax1.grid()

    # Add legend
    prev_handles, prev_labels = fig_1_ax1.get_legend_handles_labels()
    prev_handles.append(lc)
    prev_labels.append(r'$d(\mathbf{x}, \mathcal{O})$')
    fig_1_ax1.legend(prev_handles, prev_labels, handler_map={lc: HandlerColorLineCollection(numpoints=4)},
                     framealpha=1,
                     loc='lower left', fontsize='20')

    # Generate plot and save
    plt.tight_layout()
    plt.savefig(plot_name + '.pdf', format="pdf", bbox_inches="tight")
    plt.savefig(plot_name + '.png', format="png", bbox_inches="tight")
    plt.show()


def draw_plot_u_shape(plot_name, gvn_bag, path_array, obs_array):
    """
    Plot python visualization of corresponding simulation
    """

    # Create decoder class
    decoder = DataDecoder()

    # Decode corresponding topic into numpy array
    decoder.gazebo_p3d_odom_msg_decoder(gvn_bag)
    decoder.ref_gvn_status_msg_decoder(gvn_bag)
    decoder.cone_controller_debug_msg_decoder(gvn_bag)

    # Calculate duration
    end_time_index = np.argwhere(decoder.cone_controller_status == 2)[0]
    duration_stamp = decoder.cone_controller_timestamp[end_time_index, :] - decoder.cone_controller_timestamp[0, :]
    duration = np.around(duration_stamp[0, 0] + duration_stamp[0, 1] * 1e-9, decimals=2)

    # Obtain start and end points
    start_point = path_array[0, :]
    end_point = path_array[-1, :]

    # Create fig
    fig_1 = plt.figure(figsize=(8, 7))
    fig_1_ax1 = fig_1.add_subplot(111)

    # Draw Obstacles
    fig_1_ax1 = draw_path2d(fig_1_ax1, obs_array, 'grey', 8, 'Obstacle', '-')

    # Draw planning path
    fig_1_ax1 = draw_path2d(fig_1_ax1, path_array, 'black', 2, 'Planning Path', '--')

    # Draw velocity arrow
    fig_1_ax1 = draw_velocity_arrow(fig_1_ax1, decoder.odom_pose)

    # ----------- Multicolor Traj ------------
    # Divide path into segments
    points = (decoder.odom_pose[:, 0:2]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Determine colormap upper and lower bounds
    norm = plt.Normalize(1, 3)
    lc = LineCollection(segments, cmap='RdYlGn_r', norm=norm)

    # Draw multicolor path indicating dIrO
    lc.set_array(decoder.sddm_boost)
    lc.set_linewidth(4)

    # Draw colorbar
    multi_color_line = fig_1_ax1.add_collection(lc)
    divider = make_axes_locatable(fig_1_ax1)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    fig_1.colorbar(multi_color_line, cax=cax)

    # ----------------------------------------

    # Draw start and end points
    fig_1_ax1.scatter(start_point[0], start_point[1], c='red', marker='*', s=320)
    fig_1_ax1.scatter(end_point[0], end_point[1], c='green', marker='*', s=320)

    # Set upper and lower limits for x and y axis
    fig_1_ax1.set_xlim(-4, 10)
    fig_1_ax1.set_aspect('equal')

    fig_1_ax1.grid()

    # Add legend
    prev_handles, prev_labels = fig_1_ax1.get_legend_handles_labels()
    prev_handles.append(lc)
    prev_labels.append(r'$d(\mathbf{x}, \mathcal{O})$')
    fig_1_ax1.legend(prev_handles, prev_labels, handler_map={lc: HandlerColorLineCollection(numpoints=4)},
                     framealpha=1,
                     loc='lower left', fontsize='20')

    fig_1_ax1.text(-2, -5, 'Finish Time: ' + str(duration) + ' secs', fontsize=15)

    # Generate plot and save
    plt.tight_layout()
    plt.savefig(plot_name + '.pdf', format="pdf", bbox_inches="tight")
    plt.savefig(plot_name + '.png', format="png", bbox_inches="tight")
    plt.show()


# Load bags containing odom information
jackal_race_max_bag_decoded = load_single_bag("/home/yiyz/test_rosbag/sim_costmap_max.bag",
                                              ["/ref_gvn_status", "/gazebo_p3d/odom", "/ref_gvn_debug",
                                               "/cone_controller/debug"])['sim_costmap_max']

jackal_race_mid_bag_decoded = load_single_bag("/home/yiyz/test_rosbag/sim_costmap_mid.bag",
                                              ["/ref_gvn_status", "/gazebo_p3d/odom", "/ref_gvn_debug",
                                               "/cone_controller/debug"])['sim_costmap_mid']

u_shape_boosted_bag_decoded = load_single_bag("/home/yiyz/test_rosbag/u_shape_boosted.bag",
                                              ["/ref_gvn_status", "/gazebo_p3d/odom", "/ref_gvn_debug",
                                               "/cone_controller/debug"])['u_shape_boosted']

u_shape_bag_decoded = load_single_bag("/home/yiyz/test_rosbag/u_shape.bag",
                                      ["/ref_gvn_status", "/gazebo_p3d/odom", "/ref_gvn_debug",
                                       "/cone_controller/debug"])['u_shape']

# Load bags containing path informathion
jackal_race_max_path_decoded = load_single_bag("/home/yiyz/ros_bag_report/optimality_vs_safety/costmap_maximum_clearance.bag",
                                               ["/path"])["costmap_maximum_clearance"]

jackal_race_mid_path_decoded = load_single_bag("/home/yiyz/ros_bag_report/optimality_vs_safety/costmap_middle.bag",
                                               ["/path"])["costmap_middle"]

# Path to pkl file containing grid map information
jackal_race_max_obstacle_path = "/home/yiyz/zhl_lib/pbf_rg/data/obs_max.pkl"

jackal_race_mid_obstacle_path = "/home/yiyz/zhl_lib/pbf_rg/data/obs_mid.pkl"

# Creat obstacle for u_shape env
u_shape_obs_connors = np.array([[-1, -1.65],
                                [-1, 1.35],
                                [7, 1.35],
                                [7, -11.65],
                                [-1, -11.65],
                                [-1, -8.65],
                                [4, -8.65],
                                [4, -1.65],
                                [-1, -1.65]])

u_shape_path = np.array([[0, 0],
                         [5.25, 0],
                         [5.25, -10],
                         [0, -10]])

draw_plot_jackal_race('mid_w_obs', jackal_race_mid_bag_decoded, jackal_race_mid_path_decoded, jackal_race_mid_obstacle_path)
draw_plot_u_shape('u_shape', u_shape_bag_decoded, u_shape_path, u_shape_obs_connors)
