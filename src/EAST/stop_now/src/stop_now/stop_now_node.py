#!/usr/bin/env python3
"""
Advanced Emergency Stop

---

Parameter table:
use_program_interrupt: whether to use emergency stop topic from upstream to trigger emergency stop

monitoring_topic: if 'use_program_interrupt' is true, then the following parameter should be provided,
and the data type of provided topic should be bool

use_human_interrupt: whether to allow human operator to trigger emergency stop, recommend to allow

hold_to_run: if set to true, then human operator need to hold certain button to prevent emergency stop from being
triggered this flag does not take effect unless use_human_interrupt is set to true

joy_axes: config which axis to use on joystick to server as emergency stop trigger

joy_buttons: this setting is ONLY VALID if 'joy_axes' is not provided, which config which button to use on joystick

---

Support Package: twist_mux
https://wiki.ros.org/twist_mux
---

UCSD ERL Y. Yi, v1.0

---
"""

import importlib

import rospy
from erl_msgs.msg import EmergencyStopMsg
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class EmergencyStop:
    def __init__(self, config_dict):
        # ------------------- Default Flags --------------------
        # Loadable params
        self.use_human_interrupt = True
        self.use_program_interrupt = False
        self.use_custom_topic = False
        # internal params
        self._upstream_connection_ready = False
        self._upstream_data_ready = False
        self.human_interrupt_issued = True
        self.program_interrupt_issued = True

        # ------------------- Config Check and Load--------------------
        if 'use_program_interrupt' in config_dict:
            self.use_program_interrupt = config_dict['use_program_interrupt']
            if 'use_custom_topic' in config_dict:
                self.use_custom_topic = config_dict['use_custom_topic']

        if 'use_human_interrupt' in config_dict:
            self.use_human_interrupt = config_dict['use_human_interrupt']

        if 'hold_to_run' in config_dict:
            self.hold_to_run = config_dict['hold_to_run']
        else:
            self.hold_to_run = True

        # program interrupt params check and load
        # if any of the configuration params are not correctly provided, then use_program_interrupt is set to False
        if self.use_program_interrupt:
            if 'monitoring_topic' not in config_dict:
                rospy.logwarn_once("[stop_now_node] ------------ USE PROGRAM INTERRUPT ------------")
                rospy.logwarn_once("[stop_now_node] ---------- BUT NO TOPIC CONFIGURATION ----------")
                self.use_program_interrupt = False
                self.hold_to_run = True
            else:
                monitoring_topic = config_dict['monitoring_topic']

            if self.use_custom_topic:
                if 'monitoring_topic_msg_module' not in config_dict or 'monitoring_topic_msg_type' not in config_dict \
                        or 'program_interrupt_variable' not in config_dict or 'program_interrupt_threshold' not in config_dict:
                    rospy.logwarn_once("[stop_now_node] ------------ USE CUSTOM TOPIC ------------")
                    rospy.logwarn_once("[stop_now_node] ----------- BUT NOT CONFIGURED -----------")
                    self.use_program_interrupt = False
                    self.hold_to_run = True
                else:
                    topic_type_module = config_dict['monitoring_topic_msg_module']
                    topic_type_type = config_dict['monitoring_topic_msg_type']
                    self.interrupt_variable_name = config_dict['program_interrupt_variable']
                    self.program_interrupt_threshold = config_dict['program_interrupt_threshold']
                    if not isinstance(self.program_interrupt_threshold, (int, bool)):
                        rospy.logwarn_once("[stop_now_node] ------------ WRONG THRESHOLD TYPE ------------")
                        self.use_program_interrupt = False
                        self.hold_to_run = True
                    _msg = importlib.import_module(topic_type_module)
                    msg = getattr(_msg, topic_type_type)
            else:
                self.interrupt_variable_name = "error_code"
                self.program_interrupt_threshold = 0

        # Check if disable both
        if not self.use_human_interrupt and not self.use_program_interrupt:
            rospy.logwarn_once("[stop_now_node] ------------ WRONG CONFIG ------------")
            rospy.logwarn_once("[stop_now_node] ------------ RUN DEFAULT ------------")
            self.use_human_interrupt = True
            self.hold_to_run = True

        # human interrupt params check and load
        if self.use_human_interrupt:
            if 'joy_axes' in config_dict:
                self.joy_axes = config_dict['joy_axes']
                self.use_axes = True
            elif 'joy_buttons' in config_dict:
                self.joy_buttons = config_dict['joy_buttons']
                self.use_axes = False
            else:
                self.joy_axes = 5
                self.use_axes = True
                rospy.logwarn_once("[stop_now_node] ------------ USE AXES 5 BY DEFAULT ------------")

        # ------------------- ROS Subscriber  --------------------
        if self.use_program_interrupt:
            if self.use_custom_topic:
                self.program_sub = rospy.Subscriber(monitoring_topic, msg, self.program_interrupt_callback, queue_size=1)
            else:
                self.program_sub = rospy.Subscriber(monitoring_topic, EmergencyStopMsg, self.program_interrupt_callback, queue_size=1)

        if self.use_human_interrupt:
            self.human_sub = rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.human_interrupt_callback,
                                              queue_size=1)

        # ------------------- ROS Publisher  --------------------
        self.stop_pub = rospy.Publisher("/e_stop", Bool, queue_size=1)

        # ------------------- Init Modules  --------------------
        self.human_interrupt_variable = None
        self.program_interrupt_variable = None
        self.stop_pub.publish(True)
        self.init_upstream()

    def init_upstream(self):
        """
        This function checks if all subscribed topics are being published normally
        """

        while (not self._upstream_connection_ready) and (not rospy.is_shutdown()):
            self._check_upstream_connections()
            rospy.sleep(0.1)  # avoid inquery too fast
            rospy.logdebug_throttle(1.0, "waiting upstream connections...")
        rospy.loginfo("[stop_now] upstream [connection] is ready, check upstream [data]...")

        while (not self._upstream_data_ready) and (not rospy.is_shutdown()):
            self._check_upstream_data()
            rospy.sleep(0.1)  # avoid inquery too fast

        rospy.loginfo_throttle(1.0, "[stop_now] upstream [data] is ready!")

        rospy.loginfo("[stop_now] Upstream Init Done!")

    def _check_upstream_connections(self):
        """
        This function checks how many topics we have obtained connection
        """

        if self.use_program_interrupt and self.use_human_interrupt:
            upstream_connection = 2
            self._upstream_connection = self.human_sub.get_num_connections() + self.program_sub.get_num_connections()
        else:
            upstream_connection = 1
            if self.use_human_interrupt:
                self._upstream_connection = self.human_sub.get_num_connections()
            else:
                self._upstream_connection = self.program_sub.get_num_connections()

        if self._upstream_connection == upstream_connection:
            self._upstream_connection_ready = True
        else:
            if self.use_human_interrupt:
                if self.human_sub.get_num_connections() < 1:
                    rospy.loginfo_throttle(1.0, "[stop_now] waiting human interruption topic...")

            if self.use_program_interrupt:
                if self.program_sub.get_num_connections() < 1:
                    rospy.loginfo_throttle(1.0, "[stop_now] waiting program interruption topic...")

            self.stop_pub.publish(True)

    def _check_upstream_data(self):
        """
        This function checks if we are getting the correct data from the subscribed topics
        """

        status = True
        if self.use_human_interrupt:
            if self.human_interrupt_variable is None:
                status = False
                rospy.loginfo_throttle(1.0, "[stop_now] waiting human interruption data...")

        if self.use_program_interrupt:
            if self.program_interrupt_variable is None:
                status = False
                rospy.loginfo_throttle(1.0, "[stop_now] waiting program interruption data...")

        if status:
            self._upstream_data_ready = True
            rospy.loginfo_once("\n[stop_now] all %d upstream data initialized !\n" % self._upstream_connection)
        else:
            self.stop_pub.publish(True)

    def program_interrupt_callback(self, msg):
        """
        This function check if emergency has been triggered by upstream programs
        """

        if self.use_custom_topic:
            self.program_interrupt_variable = getattr(msg, self.interrupt_variable_name)
        else:
            temp_code = getattr(msg, self.interrupt_variable_name)
            self.program_interrupt_variable = -float(temp_code.data)

        if isinstance(self.program_interrupt_variable, int):
            # program interrupt variable int case, compare with interrupt threshold
            self.program_interrupt_issued = self.program_interrupt_variable < self.program_interrupt_threshold
        else:
            # program interrupt variable bool case
            self.program_interrupt_issued = self.program_interrupt_variable

        if self.program_interrupt_issued:
            if not self.use_custom_topic:
                rospy.logwarn_throttle(0.5, "[stop_now] ------- upstream emergency stop message ------- \n"
                                            "[stop_now] --------------------- %s ----------------------"
                                       % msg.error_msg.data)

    def human_interrupt_callback(self, msg):
        """
        This function check if emergency has been triggered by human operator
        """

        if self.use_axes:
            self.human_interrupt_variable = -msg.axes[self.joy_axes]
        else:
            self.human_interrupt_variable = msg.buttons[self.joy_buttons]

        if self.hold_to_run:
            self.human_interrupt_issued = self.human_interrupt_variable <= 0
        else:
            self.human_interrupt_issued = self.human_interrupt_variable > 0

        if self.human_interrupt_issued:
            rospy.logwarn_throttle(0.5, "[stop_now] ------- human emergency stop triggered -------")

    def publish_check(self):
        """
        This function decided whether to publish True in emergency stop message based on user configuration
        """

        if self.use_human_interrupt and self.use_program_interrupt:
            # both human_interrupt and program_interrupt are use, no emergency stop if both of them are NOT triggered
            if not self.human_interrupt_issued and not self.program_interrupt_issued:
                self.stop_pub.publish(False)
            else:
                self.stop_pub.publish(True)
        else:
            # one of human_interrupt and program_interrupt is used, emergency stop triggered as demand
            if self.use_human_interrupt:
                # human interrupt case
                if self.human_interrupt_issued:
                    self.stop_pub.publish(True)
                else:
                    self.stop_pub.publish(False)
            else:
                # program interrupt case
                if self.program_interrupt_issued:
                    self.stop_pub.publish(True)
                else:
                    self.stop_pub.publish(False)


if __name__ == '__main__':

    config_table = {}

    try:
        rospy.init_node('stop_now')
        rospy.loginfo("Emergency Stop Started!\n")

        # ------------------- Default Config Table  --------------------
        # use use_program_interrupt
        config_table['use_program_interrupt'] = rospy.get_param('~use_program_interrupt', True)
        # use use_human_interrupt, current human interrupt method: joystick
        config_table['use_human_interrupt'] = rospy.get_param("~use_human_interrupt", False)
        # use custom program interrupt topic
        config_table['use_custom_topic'] = rospy.get_param("~use_custom_topic", False)
        # topic for which contains status, we would like to check and trigger emergency stop signal
        config_table['monitoring_topic'] = rospy.get_param("~monitoring_topic", "/ref_gvn_emergency")

        if config_table['use_custom_topic']:
            # module which contains corresponding topic message type, for import use
            config_table['monitoring_topic_msg_module'] = rospy.get_param("~monitoring_topic_msg_module",
                                                                          "erl_msgs.msg._RefGvnMsg")
            # message type for monitoring topic, for import use
            config_table['monitoring_topic_msg_type'] = rospy.get_param("~monitoring_topic_msg_type", "RefGvnMsg")
            # which variable we wish to use in monitoring topic
            config_table['program_interrupt_variable'] = rospy.get_param("~program_interrupt_variable", "gvn_status")
            # less than this value will trigger e-stop
            config_table['program_interrupt_threshold'] = rospy.get_param("~program_interrupt_threshold", 2)

        # whether operator needs to hold certain button on joystick to enable motor on robot
        config_table['hold_to_run'] = rospy.get_param("~hold_to_run", False)
        # This config specify which axis of joystick we use to serve as emergency stop button
        # If joy_axes is specified, then it will use the assigned axes as emergency stop button
        # NO MATTER whether joy_buttons is provided or not
        # PS4 trigger R2 is joy_axes 5, primary config using R2, you can set alternative axes
        config_table['joy_axes'] = rospy.get_param("~joy_axes", 5)
        # This config specify which button on joystick we use on joystick to serve as emergency stop button
        # ONLY VALID if joy_axes is not provided
        # PS4 button square is joy_buttons 3, primary config using square, you can set alternative buttons
        config_table['joy_buttons'] = rospy.get_param("~joy_buttons", 3)

        emergency_stop = EmergencyStop(config_dict=config_table)
        rate = rospy.Rate(60)

        # rate = rospy.Rate(ref_gvn_ros.core.rate)
        while not rospy.is_shutdown():
            emergency_stop.publish_check()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
