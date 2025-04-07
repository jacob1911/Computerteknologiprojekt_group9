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
import smbus


def convert_data(data, index):
    return data[index + 1] + data[index] / 256

threshold = 0.38

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')

        self.prev_array = []
        self.scan_ranges = []
        self.divisions = 20
        self.has_scan_received = False

        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(0x44, 0x01, 0x05)  # Configure the sensor
        self.color_detect = ""
        self.counter_green = 0
        self.counter_red = 0
        self.counter_blue = 0
        self.color_detected = False

        
        self.collide_distance = 0.16
        self.stop_distance = 0.2
        self.start_turning = 0.7
        self.max_speed = 0.0 # 0.2
        
        self.turn_thresh = 5.
        self.duration = 45.
        self.l_d = 1.
        self.r_d = -1.

        self.speed_acc = 0.
        self.loop_count = 0.
        self.collision_counter = 0
        self.colliding = False
        self.collide_length_scan_ranges = 160

        self.go = False

        self.tele_twist = Twist()
        self.tele_twist.linear.x = self.max_speed
        self.max_angle = 0 # 1.28
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

    # ____________________________________RGB SENSOR____________________________________
   
    def get_and_update_color(self):
            
            data = self.bus.read_i2c_block_data(0x44, 0x09, 6)
            # Convert the data to green, red and blue int values
            # Insert code here
            
            # Output data to the console RGB values
            # Uncomment the line below when you have read the red, green and blue values
            # print("RGB(%d %d %d)" % (red, green, blue))
            green = convert_data(data, 0)
            red = convert_data(data, 2)
            blue = convert_data(data, 4)

            blue = blue * 2.35

            total = green + red + blue
            
            green = green / total
            red = red / total
            blue = blue / total

            if(green > threshold):
                self.color_detect = "green"
            elif(red > threshold):
                self.color_detect = "red"
            elif(blue > threshold):
                self.color_detect = "blue"
            else:
                self.color_detect = "no color detected"


            if not self.color_detected:
                if self.color_detect == "green":
                    self.counter_green += 1
                    self.color_detected = True
                    
                elif self.color_detect == "red":
                    self.counter_red += 1
                    self.color_detected = True

                elif self.color_detect == "blue":
                    self.counter_blue += 1
                    self.color_detected = True
                
            else:
                # print("im currently at a color")
                if not self.color_detect in ["green", "red", "blue"]:
                    self.color_detected = False
            
                
            # print("\ncolor detected: ", self.color_detect)

    # ____________________________________RGB SENSOR____________________________________


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
        # current_time = self.get_clock().now().nanoseconds / 1e9
        
        # if int(current_time - self.start_time) > self.run_time:
        #     self.get_logger().info("Time limit reached. Shutting down node.")
        #     rclpy.shutdown()
        #     return
        
        if self.has_scan_received:
            self.detect_obstacle()

        self.get_and_update_color()
        

    # Lineær udvikling af konstanten
    def angle_factor(self,front_d):
        x = abs(self.start_turning - front_d)
        return x / (self.start_turning-self.stop_distance)
    
    def speed_factor(self, front_d):
        if(front_d > self.start_turning):
            return 1.
        return front_d / (self.start_turning - self.stop_distance)



    # Afgører om robotten skal dreje højre eller venstre om obstacle
    def set_rot_d(self, r_dist, l_dist): 

        for i in range(self.divisions // 4 - 1):
            min_r = min(r_dist[i], r_dist[i+1])
            min_l = min(l_dist[i], l_dist[i+1])
            if(self.stop_distance * 1.5 < max(min_l, min_r)):
                if(min_l < min_r):
                    self.turn_factor = self.r_d
                    return
                else:
                    self.turn_factor = self.l_d
                    return
            if(self.start_turning / 1.5 < min(r_dist[i], r_dist[i+1])):
                self.turn_factor = self.r_d
                return
            elif(self.start_turning / 1.5 < min(l_dist[i], l_dist[i+1])):
                self.turn_factor = self.l_d
                return
        
        min_l = min(l_dist[1], l_dist[2])
        min_r = min(r_dist[1], r_dist[2])
        if(abs(min_l - min_r) < self.turn_thresh):
            self.turn_factor = self.l_d
        elif(min_l < min_r):
            self.turn_factor = self.r_d

        else:
            self.turn_factor = self.l_d
    
    def stop_running(self):
        # Printer gennemsnitshastighed
        print(f"Average Linear Speed: {self.speed_acc / self.loop_count}")
        print(f"Collision counter: {self.collision_counter}")
        print(f"Color detected:\n Red: {self.counter_red}, Green: {self.counter_green}, Blue: {self.counter_blue}")
        
        # Stopper robotten
        twist = Twist()
        twist.angular.z = 0.
        twist.linear.x = 0.
        self.cmd_vel_pub.publish(twist)
            

    def set_distances(self):
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
        
        return right_distances, left_distances

    def detect_obstacle(self):

        
        right_distances, left_distances = self.set_distances()
        
        
        front_distance = min(left_distances[0], right_distances[0])
        
        under_16 = 0
        for i in self.scan_ranges:
            if i < 0.16:
                under_16 += 1
        
        len_scan_ranges = len(self.scan_ranges)
        rel_16 = under_16 / len_scan_ranges
        print(f"Readings under 16: {under_16}. Relative readings under 16: {rel_16}. Length of scan ranges: {len_scan_ranges}.")

        twist = Twist()
        
        if len_scan_ranges > self.collide_length_scan_ranges and self.colliding==False:
            self.collision_counter +=1
            self.colliding = True
        else:
            print(('Im colliding'))
            if len_scan_ranges < self.collide_length_scan_ranges:
                self.colliding = False
        
        # 1) Checks if it should stop
        if front_distance < self.stop_distance:
            twist.linear.x = 0.0
            self.set_rot_d(right_distances, left_distances)
            twist.angular.z = self.max_angle * self.turn_factor
            self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)

        # 2) Checks how hard it should turn
        elif front_distance < self.start_turning:
            twist.linear.x = self.max_speed * self.speed_factor(front_distance)
            # print(f"Linear speed: {twist.linear.x}, front distance: {front_distance}")
            self.set_rot_d(right_distances, left_distances)
            twist.angular.z = self.max_angle * self.angle_factor(front_distance) * self.turn_factor
                  
        else:
            twist = self.tele_twist

        self.speed_acc += twist.linear.x
        self.loop_count += 1
        self.cmd_vel_pub.publish(twist)


        
        

        


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()