#!/usr/bin/env python3
""" Low level velocity controller for unicycle-like robot. 

Interfaces:
    Input:
            desired robot states (z*) from ref_gvn node z* = zg
            current robot states (z) from odometry 
    Output:
            velocity command for mobile platform or simulated dynamics

"""

import rospy
import numpy as np
from erl_unicycle_controller.cone_controller import ConeController
from erl_unicycle_controller.cone_bidirectional_controller import ConeBDController
from erl_unicycle_controller.post import UnicycleControlPostprocess
from erl_unicycle_controller.pre import UnicycleControlPreprocess
from ref_gvn_utils.geometry_utils import wrap_angle_pmp


class UnicycleControllerWrapper:
    def __init__(self, config_dict=None):
        """ Init UnicycleControllerWrapper class.

            This controller subscribes:
                odom (from localization)
                desired robot states (from high level controller, i.e., ref_gvn)
            Publish:
                desired velocity / body twist 
        """
        self.preprocessor = None
        self.core = None
        self.postprocessor = None
        self.cmd_pub = None

        # loading external config parameters
        self._config_dict = config_dict

        # -------------------- Constants -------------------------

        self.ctrl_params = self._config_dict["ctrl_params"]
        self.ctrl_limits = self._config_dict["ctrl_limits"]

        self.enable_sddm_boost = self._config_dict['sddm_boost']

        self.bi_direction = self._config_dict['bi_direction']

        rospy.logwarn("self.ctrl_params %s" % self.ctrl_params)
        rospy.logwarn("self.ctrl_limits %s" % self.ctrl_limits)
        rospy.logwarn("self.sddm_boost %s" % self.ctrl_limits)

        # ------------------- Init Modules  --------------------
        self.init_preprocessor()
        self.init_core()
        self.init_postprocessor()

        # set numpy array console print precision = 2
        np.set_printoptions(formatter={'float': '{: 0.2f}'.format})
        rospy.loginfo("UNICYCLE CONTROL NODE INIT SUCCESSFUL!")

    def init_preprocessor(self):
        """
        Init preprocessor of unicycle control node.
        """
        self.preprocessor = UnicycleControlPreprocess()

    def init_core(self):
        """
        Init unicycle controller. For each controller the interface might be different. 
        More details in controller itself.
        """
        if self.bi_direction:
            self.core = ConeBDController(ctrl_params=self.ctrl_params)
        else:
            self.core = ConeController(ctrl_params=self.ctrl_params)

    def init_postprocessor(self):
        """
        Init preprocessor of node.
        """
        self.postprocessor = UnicycleControlPostprocess(ctrl_limits=self.ctrl_limits)
        self.cmd_pub = self.postprocessor.cmd_vel_pub

    def show_debug_info(self, z, z_dsr, v, w):
        """
        Display more debug info. 
        """
        err_dist_norm = np.linalg.norm(z_dsr[0:2] - z[0:2])
        err_angle_norm = np.abs(np.rad2deg(wrap_angle_pmp(z_dsr[2] - z[2])))
        rospy.logdebug_throttle(1, "[unicycle controller update]")
        rospy.logdebug_throttle(1, "z =     [%.2f, %.2f, %.2f]" % (z[0], z[1], z[2]))
        rospy.logdebug_throttle(1, "z_dsr = [%.2f, %.2f, %.2f]" % (z_dsr[0], z_dsr[1], z_dsr[2]))

        if not self.core.goal_pose_reached_announced:
            msg1 = "[llc node] [e_dist, e_angle (deg)] = [%.3f, %.3f]" % (err_dist_norm, err_angle_norm)
            msg1 += "\t[v, w] = [%.2f, %.2f]" % (v, w)
            rospy.loginfo_throttle(1, msg1)

    def update(self):
        """
        Update loop as follows:
            1. collect latest data from preprocessor (callback automatically)
            2. execute update loop using core
            3. sending command to downstream via post-processor
        """

        z = self.preprocessor.np_z
        z_dsr = self.preprocessor.np_z_dsr

        # control gain boost from directional distance metric / euclidean distance metric
        if self.enable_sddm_boost:
            sddm_boost_val = self.preprocessor.sddm_boost
        else:
            sddm_boost_val = 1.0

        v, w = self.core.generate_control(z=z, z_dsr=z_dsr, sddm_boost=sddm_boost_val, debug=False)

        # show debug info in ros terminal
        self.show_debug_info(z=z, z_dsr=z_dsr, v=v, w=w)
        # send out velocity command
        self.postprocessor.send_cmd(v_dsr=v, w_dsr=w, clip_ctrl=True)
        # publish debug message
        self.postprocessor.pub_debug(z, z_dsr, sddm_boost, self.core.status, self.ctrl_limits)


if __name__ == '__main__':
    try:
        rospy.init_node('unicycle_controller')
        rospy.loginfo("[unicycle_controller] Started!\n")

        # loading parameters
        ctrl_freq = rospy.get_param("~ctrl_freq", 50.0)
        kv = rospy.get_param("~kv", 0.5)
        kw = rospy.get_param("~kw", 1.5)

        # control limit
        v_min = rospy.get_param("~v_min", -2.0)
        v_max = rospy.get_param("~v_max", 2.0)

        w_min = rospy.get_param("~w_min", 1.0)
        w_max = rospy.get_param("~w_max", -1.0)

        sddm_boost = rospy.get_param("~sddm_boost", True)

        bi_direction = rospy.get_param("~bi_direction")

        config_dictionary = {
            'ctrl_params': {'kv': kv, 'kw': kw},
            'ctrl_limits': {'v_min': v_min, 'v_max': v_max, 'w_min': w_min, 'w_max': w_max},
            'sddm_boost': sddm_boost,
            'bi_direction': bi_direction,
        }

        unicycle_controller = UnicycleControllerWrapper(config_dictionary)
        rate = rospy.Rate(ctrl_freq)

        while not rospy.is_shutdown():
            unicycle_controller.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
