#!/usr/bin/env python
import math
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
import numpy as np
from lane_controller.new_controller import LaneController

class LaneControllerNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary
        self.parameters['~v_bar'] = None
        self.parameters['~k_d'] = None
        self.parameters['~k_theta'] = None
        self.parameters['~k_Id'] = None
        self.parameters['~k_Iphi'] = None
        self.parameters['~theta_thres'] = None
        self.parameters['~d_thres'] = None
        self.parameters['~d_offset'] = None
        self.parameters['~use_rad_lim'] = None
        self.parameters['~min_rad'] = None
        self.parameters['~wheel_distance'] = None
        self.parameters['~velocity_to_m_per_s'] = None
        self.parameters['~omega_to_rad_per_s'] = None
        self.parameters['~integral_bounds']= None
        self.parameters['~d_resolution'] = None
        self.parameters['~phi_resolution'] = None
        self.parameters['~omega_ff'] = None
        self.parameters['~omega_max'] = None
        self.parameters['~omega_min'] = None
        self.parameters['~verbose'] = None
        self.updateParameters()

        #self.flag_dict = {"fleet_planning_lane_following_override_active": False}

        # Initialize variables
        self.fsm_state = None
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.active = True
        self.last_s = None
        self.stop_line_distance = None
        self.stop_line_detected = False
        self.controller = LaneController(self.parameters)

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber("~lane_pose",
                                                 LanePose,
                                                 self.cbAllPoses,
                                                 "lane_filter",
                                                 queue_size=1)
        self.sub_intersection_navigation_pose = rospy.Subscriber("~intersection_navigation_pose",
                                                                 LanePose,
                                                                 self.cbAllPoses,
                                                                 "intersection_navigation",
                                                                 queue_size=1)
        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed",
                                                        WheelsCmdStamped,
                                                        self.updateWheelsCmdExecuted,
                                                        queue_size=1)
        self.sub_actuator_limits = rospy.Subscriber("~actuator_limits",
                                                    Twist2DStamped,
                                                    self.updateActuatorLimits,
                                                    queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch,  queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
        self.sub_stop_line = rospy.Subscriber("~stop_line_reading",
                                              StopLineReading,
                                              self.cbStopLineReading,
                                              queue_size=1)
        self.log("Initialized!")

    def cbStopLineReading(self, msg):
        self.stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2 + msg.stop_line_point.z**2)
        self.stop_line_detected = msg.stop_line_detected

    def cbAllPoses(self, input_pose_msg, pose_source):
        # TODO add active stuff && sleepMaintenance
        self.pose_msg_dict[pose_source] = input_pose_msg

        # TODO do we keep this?
        # if self.pose_initialized:
        #     v_ref_possible_default = self.v_ref_possible["default"]
        #     v_ref_possible_main_pose = self.v_ref_possible["main_pose"]
        #     self.v_ref_possible.clear()
        #     self.v_ref_possible["default"] = v_ref_possible_default
        #     self.v_ref_possible["main_pose"] = v_ref_possible_main_pose

        if self.parameters['~verbose'] == 2:
            self.log("Pose source: %s" % pose_source)

            self.pose_msg = input_pose_msg
            self.pose_initialized = True

#        # TODO do we keep this?
#        if self.flag_dict["fleet_planning_lane_following_override_active"] == True:
#            if "fleet_planning" in self.pose_msg_dict:
#                self.pose_msg.d_ref = self.pose_msg_dict["fleet_planning"].d_ref
#                self.v_ref_possible["fleet_planning"] = self.pose_msg_dict["fleet_planning"].v_ref

        if self.pose_msg != self.prev_pose_msg and self.pose_initialized:
            self.getControlAction(self.pose_msg)

    def cbWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd

    def updateActuatorLimits(self, msg_actuator_limits):
        # TODO make this a service!
        self.actuator_limits = msg_actuator_limits

    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)

    def getControlAction(self, pose_msg):

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - pose_msg.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs / 1e9

        # Compute errors
        d_err = pose_msg.d - self.parameters['~d_offset']
        phi_err = pose_msg.phi

        # We cap the error if it grows too large
        if math.fabs(d_err) > self.parameters['~d_thres']:
            self.log("d_err too large, thresholding it!", 'error')
            d_err = d_err / math.fabs(d_err) * self.parameters['~d_thres']

        current_s = rospy.Time.now().to_sec()

        dt = None
        if self.last_ms is not None:
            dt = (current_s - self.last_s)

        wheels_cmd_exec = [self.wheels_cmd_executed.vel_left, self.wheels_cmd_executed.vel_right]
        omega = self.controller.compute_control_action(d_err, phi_err, dt, wheels_cmd_exec)

        if self.stop_line_detected:
            # Linearly decrease speed if a red line is detected
            v = self.controller.compute_velocity(self.stop_line_distance)
        else:
            v = self.parameters['~v_bar']

        # Initialize car control msg use header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        # We only have a SISO controller, v is kept fix.
        car_control_msg.v = v * self.parameters['~velocity_to_m_per_s']

        # For feedforward action (i.e. during intersection navigation)
        # TODO make sure that unicorn intersection reset this to zero
        omega += self.parameters['~omega_ff']
        if omega > self.parameters['~omega_max']:
            omega = self.parameters['~omega_max']
        if omega < self.parameters['~omega_min']:
            omega = self.parameters['~omega_min']
        car_control_msg.omega = omega

        self.publishCmd(car_control_msg)
        self.last_s = current_s

    def updateParameters(self, event=None, verbose=True):
        super(LaneControllerNode, self).updateParameters()
        self.controller.update_parameters(self.parameters)

    def cbMode(self, fsm_state_msg):
        self.fsm_state = fsm_state_msg.state    # String of current FSM state


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name='lane_controller_node')
    # Keep it spinning
    rospy.spin()
