#!/usr/bin/env python3


import numpy as np
from geometry_msgs.msg import Point
from ref_gvn_utils.geometry_utils import transform_goal2d_w2b
from ref_gvn_utils.ros_marker_utils import create_actor2d


class IcecreamConeVis:
    """
    class for ice cream cone shape robot motion prediction set display in Rviz.
    """
    def __init__(self, rbt_pose, gvn_loc2, bi_direction=False):
        """
        Input:
            @bi_direction: if False, use Euclidean Ball for backward goals
        """
        self.bi_direction = bi_direction
        self.get_ic_params(rbt_pose, gvn_loc2)
        pass

    def check_back_goal(self):
        """
        compute goal position in robot body frame, check if goal falls onto the backward half plane
        INPUT: rbt_pose - 1D-array [x1, x2, x_theta] (robot pose in world frame)
               gvn_loc2 - 1D-array [g1, g2] (governor 2d-position in world frame)
        """
        # convert governor position coordinates from WF to BF
        gvn_bf = transform_goal2d_w2b(self.rbt_pose, self.gvn_loc2)
        if gvn_bf[0] < 0:
            back_goal = True
        else:
            back_goal = False
        return back_goal

    def get_ic_params(self, rbt_pose, gvn_loc2):
        """
        generate coordinates of two tangent 2d-points for ice cream cone in robot Body Frame.
        INPUT: rbt_pose - 1D-array [x1, x2, x_theta] (robot pose in world frame)
               gvn_loc2 - 1D-array [g1, g2] (governor 2d-position in world frame)
               back_goal - if True, goal on back hafl plane (only used for non-bidirection controller)
        """
        self.rbt_pose = rbt_pose
        self.gvn_loc2 = gvn_loc2

        if not self.bi_direction:
            back_goal = self.check_back_goal()
        else:
            back_goal = False

        # convert governor position coordinates from WF to BF
        gvn_bf = transform_goal2d_w2b(rbt_pose, gvn_loc2)
        # compute tracking error (distance & heading)
        tracking_error = np.linalg.norm(gvn_bf)
        heading_error_bf = np.arctan2(gvn_bf[1], gvn_bf[0])

        # compute coords for 2 tangent points (tanpt1, tanpt2)
        tanpt1_x = tracking_error * np.cos(heading_error_bf)
        tanpt2_x = tanpt1_x * np.cos(2 * heading_error_bf)
        tanpt2_y = tanpt1_x * np.sin(2 * heading_error_bf)

        if back_goal:
            icball_radius = tracking_error
            tpt3d1 = Point(0.0, 0.0, 0.0)
            tpt3d2 = Point(0.0, 0.0, 0.0)
        else:
            icball_radius = tracking_error * np.abs(np.sin(heading_error_bf))
            tpt3d1 = Point(tanpt1_x, 0.0, 0.0)
            tpt3d2 = Point(tanpt2_x, tanpt2_y, 0.0)
        zpos3d = Point(0.0, 0.0, 0.0)

        return icball_radius, tpt3d1, tpt3d2, zpos3d

    def create_icball_marker(self):
        """
        Create marker for predicted reachable set centered at goal position (ice cream ball).
        Green ball
        """
        z = self.rbt_pose
        zg = self.gvn_loc2

        radius, _, _, _, = self.get_ic_params(z, zg)
        icball_actor = create_actor2d(
            id=101,
            type=3,
            loc2d=zg[0:2],
            disp_h=0.1,
            color_rgba=[0.0, 0.4, 0.0, 0.5],
            scale_xyz=[2 * radius, 2 * radius, 0.1],
        )
        icball_actor.action = icball_actor.ADD
        return icball_actor


    def create_iccone_marker(self):
        """
        Create marker for predicted reachable set initialed at current position (ice cream cone).
        Green line-strip
        """
        z = self.rbt_pose
        zg = self.gvn_loc2

        _, tpt3d1, tpt3d2, zpos3d = self.get_ic_params(z, zg)
        iccone_actor = create_actor2d(
            id=111,
            type=4,
            loc2d=z[0:2],
            disp_h=0.1,
            color_rgba=[0.0, 0.4, 0.0, 0.5],
            scale_xyz=[0.1, 0.0, 0.0],
            yaw=z[2]
        )

        iccone_actor.points = [tpt3d1, zpos3d, tpt3d2]
        iccone_actor.action = iccone_actor.ADD
        return iccone_actor
