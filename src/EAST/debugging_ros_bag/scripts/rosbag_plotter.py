#!/usr/bin/env python3
"""
Rosbag Data Plotter

---

UCSD ERL Y. Yi, v1.0

---
"""

import bagpy
import os
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np


def load_entire_dir(dir_str, topic_list):
    dir_list = os.listdir(dir_str)
    bag_list = []
    data_bag = {}

    for item in dir_list:
        if item.endswith('.bag'):
            bag_list.append(item)

    for bag in bag_list:
        decoded_bag = bagreader(dir_str + "/" + bag)
        data_bag[bag.rsplit(".")[0]] = {}
        for topic in topic_list:
            data_bag[bag.rsplit(".")[0]][topic.rsplit("/")[-2] + "_" + topic.rsplit("/")[-1]] = pd.read_csv(
                decoded_bag.message_by_topic(topic))


def load_single_bag(path_to_bag, topic_list):
    data_bag = {}

    decoded_bag = bagreader(path_to_bag)

    data_bag[(path_to_bag.rsplit("/")[-1]).rsplit(".")[0]] = {}

    for topic in topic_list:
        data_bag[(path_to_bag.rsplit("/")[-1]).rsplit(".")[0]][
            topic.rsplit("/")[-2] + "_" + topic.rsplit("/")[-1]] = pd.read_csv(decoded_bag.message_by_topic(topic))

    return data_bag


def draw_path2d(ax, waypoints, lc='blue', lw=2, lstr='path', ps='-'):
    """ draw 2d path consists of waypoints (num_pts, 3)
    """
    ax.plot(waypoints[:, 0], waypoints[:, 1], c=lc, ls=ps, label=lstr, lw=lw)
    return ax


def draw_traj(ax, pose_2d, color='blue', line_width=4, line_str='Trajectory', line_style='-', arrow_color='magenta', arrow_scale=0.5, arrow_head_width=0):
    rbt_pose_diff = np.diff(pose_2d[:, 0:2], axis=0)
    rotation_matrix = np.array([[0, -1],
                                [1, 0]])

    rbt_pose_diff_perp = (rotation_matrix @ rbt_pose_diff.T).T

    ax = draw_path2d(ax, pose_2d[:, 0:2], color, line_width, line_str, line_style)

    ax.quiver(pose_2d[0:-1:2, 0], pose_2d[0:-1:2, 1], rbt_pose_diff_perp[0::2, 0],
              rbt_pose_diff_perp[0::2, 1], color=arrow_color, scale=arrow_scale, headwidth=arrow_head_width)

    return ax


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


bag_decoded = load_single_bag("/home/yiyz/test_rosbag/u_shape.bag", ["/ref_gvn_status", "/gazebo_p3d/odom", "/ref_gvn_debug"])['u_shape']

decoder = DataDecoder()
decoder.gazebo_p3d_odom_msg_decoder(bag_decoded)

# draw obstacle
obs_connors = np.array([[-1, -1.65],
                        [-1, 1.35],
                        [7, 1.35],
                        [7, -11.65],
                        [-1, -11.65],
                        [-1, -8.65],
                        [4, -8.65],
                        [4, -1.65],
                        [-1, -1.65]])

way_points = np.array([[0, 0],
                       [5.25, 0],
                       [5.25, -10],
                       [0, -10]])

start_point = np.array([0, 0])
end_point = np.array([0, -10])

fig_1 = plt.figure(figsize=(8, 8))
fig_1_ax1 = fig_1.add_subplot(111)

fig_1_ax1 = draw_path2d(fig_1_ax1, obs_connors, 'grey', 8, 'Obstacle', '-')
fig_1_ax1 = draw_path2d(fig_1_ax1, way_points, 'green', 2, 'Path', '-')
fig_1_ax1 = draw_traj(fig_1_ax1, decoder.odom_pose)

fig_1_ax1.scatter(start_point[0], start_point[1], c='red', marker='*', s=320)
fig_1_ax1.scatter(end_point[0], end_point[1], c='green', marker='*', s=320)

fig_1_ax1.set_xlim(-4, 10)
fig_1_ax1.set_aspect('equal')

plt.tight_layout()
plt.show()
