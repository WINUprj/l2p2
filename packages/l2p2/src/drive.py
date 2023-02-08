#!/usr/bin/env python3

"""
TODO: Shutdown nodes (send shutdown request to odometry node).
"""

import math
import os
import time

import numpy as np
import rospy
from std_msgs.msg import Float32, String

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.srv import ChangePattern


class DriverNode(DTROS):
    def __init__(self, node_name):
        super(DriverNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        
        # Static variables
        self._veh = os.environ["VEHICLE_NAME"]
        self._rate = rospy.Rate(100)
        self._robot_width_half = 0.05

        # Values to keep track on
        self.total_distance = 0.0   # Total distance progressed by robot
        # self.average_distance = 0.0     # Average absolute distance progressed by each wheel
        self.total_ang = 0.0
        self.left_distance = 0.0    # Total distance progressed by left wheel
        self.right_distance = 0.0   # Total distance progressed by right wheel
        self.is_rot = False     # Whether robot is going to perform rotation or not
        
        # Initial robot frame and world frame
        self.rf = np.zeros((3, 1))
        self.wf = np.zeros((3, 1))
        self.wf[:2, 0] = 0.32

        # Service client
        rospy.wait_for_service(f"/{self._veh}/led_emitter_node/set_custom_pattern")
        
        self.srv_led = rospy.ServiceProxy(
            f"/{self._veh}/led_emitter_node/set_pattern",
            ChangePattern
        )

        # Publisher
        self.pub_vel = rospy.Publisher(
            f"{self._veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=10
        )
        self.pub_executed_cmd = rospy.Publisher(
            f"/{self._veh}/wheels_driver_node/wheels_cmd_executed",
            WheelsCmdStamped,
            queue_size=10
        )
        
        # Subscribers
        self.delta_dist_left = rospy.Subscriber(
            f"{self._veh}/odometry_node/left_wheel_delta",
            Float32,
            self.cb_param_update,
            callback_args="left"
        )
        self.delta_dist_right = rospy.Subscriber(
            f"{self._veh}/odometry_node/right_wheel_delta",
            Float32,
            self.cb_param_update,
            callback_args="right"
        )
    
    def reset_variables(self):
        """Reset all tracking variables to 0.
        
        Arguments
        ---------
        None
        
        Returns
        -------
        None
        """
        self.total_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.total_ang = 0.0

    def set_led_color(self, pattern):
        msg = String()
        msg.data = pattern
        self.srv_led(msg)

    def get_inv_rot_matrix(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])
        
    def to_world_frame(self, theta):
        """Convert robot frame to world frame."""
        self.wf = self.get_inv_rot_matrix(theta) * self.rf

    def cb_param_update(self, msg, wheel):
        """Update distance parameters based on the subscriber's feedback.
        
        Arguments
        ---------
        msg: Float32
            Change of distance on either wheel.
        wheel: str
            Indicator of which wheel has been called. ["left", "right"]
        """
        assert wheel in ["left", "right"]

        if wheel == "right":
            self.right_distance += msg.data
            self.total_distance = (self.left_distance + self.right_distance) / 2.
            self.total_ang = (abs(self.left_distance) + abs(self.right_distance)) / (2 * self._robot_width_half)
        else:
            self.left_distance += msg.data
    
    def send_msg(self, msg):
        """Send the message indicating velocity to suitable topic.
        
        Arguments
        ---------
        msg: WheelsCmdStamped
            Velocity message that we are feeding into a topic.
        """
        self.pub_vel.publish(msg)
        self.pub_executed_cmd.publish(msg)

    def stop(self):
        """Method to stop the robot's movement."""
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = 0.0
        msg.vel_right = 0.0

        self.send_msg(msg)

    def straight(self, distance, vel_left=0.4, vel_right=0.4, offset=0):
        """Method to move the robot in straight line for desired distance.
        
        Arguments
        ---------
        distance: float
            Desired distance for robot to move straight. In meters.
        vel_left: float
            Velocity of left wheel.
        vel_right: float
            Velocity of right wheel.
        """
        assert (vel_left > 0.0 and vel_right > 0.0) or (vel_left < 0.0 and vel_right < 0.0)
        
        # Reset the variables
        self.reset_variables()
        
        # Construct message
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        
        # Move until distance reaches `distance`
        while abs(self.total_distance) < abs(distance) - offset:
            msg.header.stamp = rospy.Time.now()
            self.send_msg(msg)
            self._rate.sleep()
        
        # Stop after reaching
        self.stop()

    def rotate(self, angle, vel_left=-0.4, vel_right=0.4, offset=0):
        """Method to rotate the robot for desired angles.
        Arguments
        ---------
        angle: float
            Desired angle for robot to rotate. In radian.
        vel_left: float
            Velocity of left wheel.
        vel_right: float
            Velocity of right wheel.
        """
        # Reset the variables
        self.reset_variables()
        
        # Construct message
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self.send_msg(msg)
        # print(self.total_ang)

        # Move until angle reaches to `angle`
        while self.total_ang < angle - offset:
            msg.header.stamp = rospy.Time.now()
            self.send_msg(msg)
            print(self.total_ang)
            self._rate.sleep()

        self.stop()


if __name__ == "__main__":
    # Initialize driver node
    driver = DriverNode("driver_node")
    # Initialize service proxy

    # driver.set_led_color("RED")
    

    ### Lab 2 Part 1
    # Straight line task
    # driver.straight(0.4, 0.6, 0.6)
    # time.sleep(2)
    # driver.straight(0.4, -0.6, -0.6)
    # time.sleep(5)

    # Roatation task
    # driver.stop()
    # time.sleep(2)
    # driver.rotate(math.pi/2, 0.6, -0.6)
    
    ### Lab 2 Part 2
    # TODO: ROS service to light the LED
    
    ### Uncomment below for running task for part 2 ###
    # # State 1.
    # time.sleep(5)

    # # State 2.
    # driver.rotate(math.pi/2, 0.4, -0.4)
    # for i in range(3):
    #     driver.straight(1.25)
    #     if i < 2:
    #         driver.rotate(math.pi/2, -0.4, 0.4)

    # # State 1.
    # time.sleep(5)

    # # State 3.
    # driver.rotate(math.pi/2, -0.4, 0.4)
    # driver.straight(1.25, 0.4, 0.4)
    # driver.rotate(math.pi, -0.4, 0.4)

    # # State 1.
    # time.sleep(5)
    
    # State 4.
    # driver.straight(0.3, 0.6, 0.6)
    driver.set_led_color("LIGHT_OFF")
    time.sleep(2)
    driver.set_led_color("GREEN")
    driver.rotate(math.pi/2, 0.55, -0.55, math.pi/2*0.2)
    driver.set_led_color("RED")
    time.sleep(5)
    for i in range(4):
        driver.set_led_color("BLUE")
        driver.straight(1, 0.6, 0.6)
        driver.set_led_color("RED")
        time.sleep(5)
        if i < 3:
            driver.set_led_color("GREEN")
            driver.rotate(math.pi/2, -0.55, 0.55, math.pi/2*0.2)
            driver.set_led_color("RED")
            time.sleep(5)
    
    driver.set_led_color("WHITE")
    driver.rotate(math.pi, -0.55, 0.55)
    driver.set_led_color("LIGHT_OFF")