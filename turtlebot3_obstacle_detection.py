#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import time as t
import numpy as np

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')


        self.scan_ranges = []
        self.divisions = 20
        self.has_scan_received = False
        
        self.stop_distance = 0.2
        self.start_turning = 0.7
        self.max_speed = 0.2
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.run_time = 10

        self.tele_twist = Twist()
        self.tele_twist.linear.x = self.max_speed
        self.max_angle = 1.28
        self.tele_twist.angular.z = 0.0
        self.turn_factor = 1.
        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        
        # Setting all bad values, inf and nan, to 3.5
        for i in range(len(self.scan_ranges)):
            cur_val = self.scan_ranges[i]
            if (not (0 < cur_val and cur_val <= 3.5)):
                self.scan_ranges[i] = 3.5
        
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if int(current_time - self.start_time) > self.run_time:
            self.get_logger().info("Time limit reached. Shutting down node.")
            rclpy.shutdown()
            return
        
        if self.has_scan_received:
            self.detect_obstacle()

    # Lineær udvikling af konstanten
    def angle_factor(self,front_d):
        x = abs(self.start_turning - front_d)
        return x / (self.start_turning-self.stop_distance)
    
    def speed_factor(self, front_d):
        if(front_d > self.start_turning):
            return 1.
        return front_d / (self.start_turning - self.stop_distance)


    # Afgører om robotten skal dreje højre eller venstre om obstacle
    def set_rot_direction(self, r_dist, l_dist):
        for i in range(self.divisions // 4 - 1):
            if(self.start_turning / 1.5 < min(r_dist[i], r_dist[i+1])):
                return -1.
            elif(self.start_turning / 1.5 < min(l_dist[i], l_dist[i+1])):
                return 1.
        
        min_l = min(l_dist[1], l_dist[2])
        min_r = min(r_dist[1], r_dist[2])
        if(min_l < min_r):
            self.turn_factor = -1.

        else:
            self.turn_factor = 1.
       
        

    

    def detect_obstacle(self):

        len_scan_range = len(self.scan_ranges)
        range_size = int(len_scan_range / self.divisions)

        left_distances = []
        for i in range(self.divisions // 4):
            min_val = min(self.scan_ranges[range_size * i: range_size * (i + 1)])
            left_distances.append(min_val)

        right_distances = []
        for i in range(self.divisions // 4):
            min_val = min(self.scan_ranges[(len_scan_range-1) - range_size * (i + 1) : (len_scan_range-1) - range_size * i])
            right_distances.append(min_val)
        
        front_distance = min([left_distances[0], right_distances[0]])

        # ## TIDLIGERE VERSION ##front_distance = min(self.scan_ranges[mid_index - front_range: mid_index + front_range]) 


        twist = Twist()
        # 1) Checks if it should stop
        if front_distance < self.stop_distance:
            twist.linear.x = 0.0
            self.set_rot_direction(right_distances, left_distances)
            twist.angular.z = self.max_angle * self.turn_factor
            self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)

        # 2) Checks how hard it should turn
        elif front_distance < self.start_turning:
            twist.linear.x = self.max_speed * self.speed_factor(front_distance)
            # print(f"Linear speed: {twist.linear.x}, front distance: {front_distance}")
            self.set_rot_direction(right_distances, left_distances)
            twist.angular.z = self.max_angle * self.angle_factor(front_distance) * self.turn_factor
                  
        else:
            twist = self.tele_twist

        self.cmd_vel_pub.publish(twist)


        
        

        


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()