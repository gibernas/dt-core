#!/usr/bin/env python
import os
import rospkg
import rospy
import yaml
import time
from math import sqrt, sin, cos

from duckietown_msgs.msg import Twist2DStamped, BoolStamped, VehiclePose, Pose2DStamped
from duckietown import DTROS


class VehicleAvoidanceControlNode(DTROS):

    def __init__(self, node_name):
        self.node_name = rospy.get_name()

        self.parameters['desired_distance'] = None
        self.parameters['minimal_distance'] = None
        self.parameters['Kp'] = None
        self.parameters['Ki'] = None
        self.parameters['Kd'] = None
        self.parameters['Kp_delta_v'] = None
        self.updateParameters()

        self.vehicle_pose_msg_temp = VehiclePose()

        # Initialization
        self.vehicle_pose_msg_temp.header.stamp = rospy.Time.now()
        self.time_temp = rospy.Time.now()
        self.v_rel = 0
        self.v = 0
        self.v_temp = 0
        self.detection = False
        self.v_error_temp = 0
        self.I = 0
        self.v_follower = 0
        self.rho_temp = 0
        self.omega = 0
        self.detection_prev = None
        self.P = 0
        self.D = 0

        # Publishers
        self.car_cmd_pub = rospy.Publisher("~car_cmd",
                                           Twist2DStamped, queue_size = 1)
        self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",
                                                    BoolStamped, queue_size=1)

        # Subscribers
        self.subscriber = rospy.Subscriber("~detection",
                                           BoolStamped, self.cb_detection,  queue_size=1)
        self.sub_vehicle_pose = rospy.Subscriber("~vehicle_pose", VehiclePose, self.cb_pose, queue_size=1)
        self.sub_car_cmd = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.cbCarCmd, queue_size=1)

    def cb_detection(self, data):

        vehicle_detected_msg_out = BoolStamped()
        vehicle_detected_msg_out.header.stamp = data.header.stamp
        vehicle_detected_msg_out.data = data.data
        self.vehicle_detected_pub.publish(vehicle_detected_msg_out)
        self.detection_prev=self.detection
        self.detection = data.data

        if not data.data:
            # self.v_gain = 1
            # self.P = 0
            self.I = 0

    def cb_pose(self, vehicle_pose_msg):
        t_init = rospy.Time.now()

        t_s = (t_init - self.t_init_temp).to_sec()

        self.vehicle_pose_msg_temp.header.stamp = vehicle_pose_msg.header.stamp

        if t_s > 4:
            self.v_rel = 0
            if vehicle_pose_msg.rho.data < self.minimal_distance:
                self.v = 0
            else:
                self.v = self.v_follower
            self.vehicle_pose_msg_temp = vehicle_pose_msg
            self.v_error_temp = 0
            self.I = 0
        else:
            self.v_rel = (vehicle_pose_msg.rho.data - self.vehicle_pose_msg_temp.rho.data)/t_s
            v_leader = self.v_follower + self.v_rel
            delta_v = (vehicle_pose_msg.rho.data - self.desired_distance)/t_s * self.Kp_delta_v
            v_des = v_leader + delta_v
            v_error = v_des - self.v_follower

            self.P = self.Kp*v_error
            self.I = self.I + self.Ki * (v_error + self.v_error_temp)/2.0*t_s
            self.D = self.Kd * (v_error + self.v_error_temp)/t_s
            self.v = self.P + self.I + self.D

            if self.v < 0 or vehicle_pose_msg.rho.data < self.minimal_distance:
                self.v = 0

            # self.rho_temp = rho
            self.v_error_temp = v_error
            self.v_temp = self.v
            self.vehicle_pose_msg_temp = vehicle_pose_msg
            # print(self.v)

        self.time_temp = t_init

    def cbCarCmd(self, car_cmd_msg):
        car_cmd_msg_current = car_cmd_msg
        car_cmd_msg_current.header.stamp = rospy.Time.now()
        if self.detection:
            car_cmd_msg_current.v = self.v
            if self.v == 0:
                car_cmd_msg_current.omega = 0

        if self.detection_prev and not self.detection:
            car_cmd_msg_current.v = 0

        if car_cmd_msg_current.v >= 0.25:
            car_cmd_msg_current.v = 0.25

        self.car_cmd_pub.publish(car_cmd_msg_current)

    def on_shutdown(self):
        self.log('Stopping and shutting down')
        # Safe stop
        safe_stop_msg = Twist2DStamped()
        safe_stop_msg.v = 0
        safe_stop_msg.omega = 0
        self.car_cmd_pub.publish(safe_stop_msg)


if __name__ == '__main__':
    # Initialize the node
    vehicle_avoidance_control_node = VehicleAvoidanceControlNode(node_name='vehicle_avoidance_control')
    # Setup proper shutdown behavior
    rospy.on_shutdown(vehicle_avoidance_control_node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()

