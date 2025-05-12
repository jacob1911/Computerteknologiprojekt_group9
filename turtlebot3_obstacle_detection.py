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
import math
import csv

# Sets up the led
from gpiozero import LED
led = LED(17)

# Function that converts data for the rgb readings
def convert_data(data, index):
    return data[index + 1] + data[index] / 256

# Thresholds for the color measurements
red_thresh = 0.46 # Red allways have to be above this
# Green and blue must be smaller than these values
green_max = 0.305
blue_max = 0.27
# If red is greater than this and blue isn't too big, a victim is also counted
red_high_thresh = 0.5
blue_high_thresh = 0.32

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')

        # Amount of time program runs
        self.duration = 120.

        # Scan range variables
        self.scan_ranges = [] # Measurements from the lidar
        self.divisions = 20 # Amount of cones the lidar readings are divided into
        self.has_scan_received = False

        # Color sensor
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(0x44, 0x01, 0x05)  # Configure the sensor
        self.victim_count = 0 # Total victims found
        self.victim_found = False # Victim is currently being detected
        # Amount of time a color has to be detected within prev_readings for
        # a victim to be detected
        self.color_count_thresh = 2
        self.prev_readings = [0, 0, 0, 0, 0, 0, 0, 0]
        # Whether or not a color is currently being detected
        self.color_detected = False
        # Amount that the robot slows down when a color is being detected
        self.color_slow_factor = 0.2
        
        # Movement variables
        self.stop_distance = 0.165 # Distance where robot stops all movement
        self.start_turning = 0.55 # Distance where robot starts turning
        self.max_speed = 0.21 # The maximum speed the robot will travel with
        # Rotation variables 
        self.max_angle = 1.28 # max rotation speed
        self.turn_factor = 1.

        self.turn_thresh = 0.05 # Difference in readings for distance to be considered valid
        self.l_d = 1. # Left rotation direction
        self.r_d = -1. # Right rotation direction

        # Variables for finding average speed
        self.speed_acc = 0.
        self.loop_count = 0.
        
        # Variables for colliding with scan_len
        self.coll_counter = 0 # Collision counter
        self.coll_front = False # If a frontal collision is detected
        self.collide_len_scans = 190 # Length of scan_ranges corresponding to a collision
        
        
        # Variables regarding edge collisions
        # Wrong readings in front left and right quadrant
        self.wrong_r_readings = 0
        self.wrong_l_readings = 0
        # Indicators for a collision taking place here
        self.coll_left = False
        self.coll_right = False
        # Amount of NaN readings indicating collisions
        self.wrong_readings_thresh = 25
        # Divison of areas where wrong measurements are being collected
        self.wrong_reading_divs = self.divisions // 2
        # Array containing the wrong measurements
        self.wrong_readings = [0] * self.wrong_reading_divs
        
        self.collision = False # General variable for if a collision has happened

        # Variables regarding the robot being stuck turning left and right
        self.stuck_thresh = 60 # Iterations, where robot has to be stuck for it to back up
        self.stuck_counter = 0 # Number of iteratons the robot currently hasn't moved
        self.im_stuck_iteration = 10 # Iterations where the robot will be backing up
        self.im_stuck = False # Indicates whether or not the robot is currently stuck
        self.loop_count_copy = 0 # Loop count when robot becomes stuck

        
        

        # Logging variables
        self.lin_speed = 0
        self.red = 0
        self.green = 0
        self.blue = 0
        self.start_time = t.time()
        self.log_path = '/home/ubuntu/log.csv'
        self.log_file = open(self.log_path, mode='w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.log_file, delimiter=';')
        self.csv_writer.writerow(['Time','Red','Green','Blue','Scan_len','Nan_L','Nan_R','Linear_Speed','Victim','Collision'])
        
        # Preexisting code
        self.tele_twist = Twist()
        self.tele_twist.angular.z = 0.0
        self.tele_twist.linear.x = self.max_speed
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

        self.timer = self.create_timer(0.05, self.timer_callback)

    # ____________________________________RGB SENSOR____________________________________
   
    def get_and_update_color(self):
            # Getting the data from the bus
            data = self.bus.read_i2c_block_data(0x44, 0x09, 6)
            
            # Gets the color values
            green = convert_data(data, 0)
            red = convert_data(data, 2)
            blue = convert_data(data, 4)

            # Corrects blue and green so that all values are equal when reading white
            blue = blue * 1.8
            green = green * 0.8
            total = green + red + blue
            
            # Normalizing the readings
            green = green / total
            red = red / total
            blue = blue / total

            # print(f"red: {red}, green: {green}, blue: {blue}")

            
            # If red is the biggest color and thresh values hold
            if (red > green and red > blue) and\
                    red > red_thresh and\
                        ((green < green_max and blue < blue_max) or \
                            (red > red_high_thresh and blue < blue_high_thresh)):
                    # Updates color detection array
                    self.prev_readings.insert(0,1)
                    self.color_detected = True
            else:
                self.prev_readings.insert(0,0)
                self.color_detected = False
            
            # Removes oldest element from color detecting array
            self.prev_readings.pop()
            
            # Amount of readings that meet the requirements in color detection array
            sum = 0
            for reading in self.prev_readings:
                sum += reading

            # If a victim was previously detected and no detections are in color array
            # the led is switched off and victim_found is set to false
            if(self.victim_found and sum == 0):
                self.victim_found = False
                led.off()

            # If a victim isn't being detected right now and two readings are in the color array
            # a victim is counted and the led is switched on
            if not self.victim_found and sum >= 2:
                print("Victim found")
                self.victim_count += 1
                self.victim_found = True
                led.on()
            
            # Color values are set for logging
            self.red = red
            self.green = green
            self.blue = blue


    # ____________________________________RGB SENSOR____________________________________


    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        # Wrong readings are reset
        self.wrong_r_readings = 0
        self.wrong_l_readings = 0
        self.wrong_readings = [0] * self.wrong_reading_divs
        
        # Setting all bad values, inf and nan, to 3.5
        for i in range(len(self.scan_ranges)):
            cur_val = self.scan_ranges[i]
            if (not (0 <= cur_val and cur_val <= 3.5)):
                self.scan_ranges[i] = 3.5
                
                # If the reading is a NaN, it is also counted as a wrong reading
                if(math.isnan(cur_val)):
                    self.wrong_readings[((int)(i / len(self.scan_ranges) * self.wrong_reading_divs))] += 1
                
        # LR wrong readings are set based on chosen regions
        for i in range(2):
            self.wrong_l_readings += self.wrong_readings[i]
            self.wrong_r_readings += self.wrong_readings [-(1+i)]
  
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg
    
    def timer_callback(self):
        # Updates color on each callback
        self.get_and_update_color()

        # Runs detect obstacle
        if self.has_scan_received:
            self.detect_obstacle()
        
        # Logs data
        self.csv_writer.writerow([t.time() - self.start_time, self.red, self.green, self.blue,\
                                    len(self.scan_ranges), self.wrong_l_readings, self.wrong_r_readings,\
                                        self.lin_speed,self.victim_found, self.collision])
        
            

       
        

    # Calculates a factor for how much the robot should turn as a linear function.
    # If front distance is lower than stop distance, this is 1
    # If front distance is greater than start turning this is 0
    def angle_factor(self,front_d):
        if(front_d > self.start_turning): return 0.
        if(front_d < self.stop_distance): return 1.
        return (-front_d + self.start_turning) / (self.start_turning-self.stop_distance)
    
    # Calculates a factor for how fast the robot should go as a linear function.
    # If front distance is lower than stop distance, this is 0
    # If front distance is greater than start turning this is 1
    def speed_factor(self, front_d):
        if(front_d > self.start_turning): return 1.
        if(front_d < self.stop_distance): return 0.
        return (front_d - self.stop_distance) / (self.start_turning - self.stop_distance)



    # Function that calculates which direction the robot should turn
    def set_rot_d(self, r_dist, l_dist): 

        # First it checks if there are any big open spaces. Checks two cones at a time
        # Two cones roughly correspond to the span of a space which the robot can enter
        for i in range(self.divisions // 4 - 1):
            # It compares the cone pair in pairs in terms of left and right.
            # Cones closer to the middle are preferred.
            # The difference between the left and right cones must be greater than some threshold.
            min_r = min(r_dist[i], r_dist[i+1])
            min_l = min(l_dist[i], l_dist[i+1])
            if(self.stop_distance * 2.2 < max(min_l, min_r) and abs(min_r - min_l) > self.turn_thresh):
                if(min_l < min_r):
                    self.turn_factor = self.r_d
                    return
                else:
                    self.turn_factor = self.l_d
                    return

        # If big open spaces aren't available, it checks if some slightly less open spaces are available
        for i in range(self.divisions // 4 - 1):
            min_r = min(r_dist[i], r_dist[i+1])
            min_l = min(l_dist[i], l_dist[i+1])
            
            if(self.stop_distance * 1.5 < max(min_l, min_r) and abs(min_r - min_l) > self.turn_thresh):
                if(min_l < min_r):
                    self.turn_factor = self.r_d
                    return
                else:
                    self.turn_factor = self.l_d
                    return
          
        # If the code above fails, cone 2 and 3 on each side is used for comparison
        # Here, the threshold is slightly lower, as no open spaces are available
        # and we thus must be closer to surrounding objects
        min_l = min(l_dist[1], l_dist[2])
        min_r = min(r_dist[1], r_dist[2])
        if(abs(min_l - min_r) > self.turn_thresh):
            if(min_l < min_r):
                self.turn_factor = self.r_d
            else:
                self.turn_factor = self.l_d
            return

        # If all else fails, the robot turns right
        self.turn_factor = self.r_d
    
    # Function that runs, when the robot has stopped running
    def stop_running(self):
        # Logs the different measured values
        print(f"Average Linear Speed: {self.speed_acc / self.loop_count}")
        print(f"Collision counter: {self.coll_counter}")
        print(f"Victims detected: {self.victim_count}")
        
        # Stops the robot from moving
        twist = Twist()
        twist.angular.z = 0.
        twist.linear.x = 0.
        self.cmd_vel_pub.publish(twist)
            
    # Generates two arrays for the left and right frontal quadrant of the robots vision
    # where each value is the smallest measured distance in the corresponding region
    def set_distances(self, len_scan_range):
        # Defines the range size from the amount of divisions and the amount of readings
        range_size = int(len_scan_range / self.divisions)

        # Sets the values in the frontal left quadrant
        left_distances = []
        for i in range(self.divisions // 4):
            min_val = min(self.scan_ranges[range_size * i: range_size * (i + 1)])
            left_distances.append(min_val)

        # Sets the values in the frontal right quadrant
        right_distances = []
        for i in range(self.divisions // 4):
            min_val = min(self.scan_ranges[(len_scan_range-1) - range_size * (i + 1) : (len_scan_range-1) - range_size * i])
            right_distances.append(min_val)
        
        return right_distances, left_distances


    # Main function that decides where to turn and how fast to go
    def detect_obstacle(self):
        len_scan_ranges = len(self.scan_ranges)
        right_distances, left_distances = self.set_distances(len_scan_ranges)
        
        # Finds the front distance as the smallest value in the two frontmost cones
        front_distance = min(left_distances[0], right_distances[0])
        
        twist = Twist()
        
        # Checks for frontal collision
        self.coll_front = len_scan_ranges < self.collide_len_scans
        
        # # Checks for collisions to the left
        self.coll_left = self.wrong_l_readings > self.wrong_readings_thresh
        
        # # Checks for collisions to the right
        self.coll_right = self.wrong_r_readings > self.wrong_readings_thresh
        
        # If a collision has happened
        if(self.coll_front or self.coll_left or self.coll_right):
            # The collision counter is updated
            if(not self.collision):
                print("Now I'm colliding")
                self.coll_counter += 1
            # Collision flag is set to true
            self.collision = True
            # Loop_count copy is reset such that it is only truly set, 
            # when the robot isn't colliding anymore
            self.loop_count_copy = self.loop_count




        # print(f"Scan range len: {len_scan_ranges}")
        # print(self.wrong_readings)
        # print(f"Wrong L: {self.wrong_l_readings}, Wrong R: {self.wrong_r_readings}")
        # print(f"Left distances: {left_distances}")
        # print(f"Right distances: {right_distances}")
        
        # If the robot is stuck somewhere turning back and forth
        if self.stuck_counter > self.stuck_thresh and not self.im_stuck:
            self.im_stuck = True
            print("im stuck")
            self.loop_count_copy = self.loop_count


        # 0) If the robot should be backing up. Happens if it is colliding or stuck

        if self.im_stuck or self.collision:
            # Linear speed is negative 
            twist.linear.x = -self.max_speed
            # It shouldn't be turning when backing up
            twist.angular.z = 0.
            # It is set to not collide or be stuck when this has run self.im_stuck_iteration times
            # It will then stop backing up
            if self.loop_count > self.loop_count_copy + self.im_stuck_iteration:
                self.im_stuck = False
                self.collision = False
            
        # 1) Checks if it should stop completely
        elif front_distance < self.stop_distance:
            twist.linear.x = 0.0 # Linear speed is set to zero
            self.set_rot_d(right_distances, left_distances) # Rotation direction is set
            twist.angular.z = self.max_angle * self.turn_factor # Rotation speed is calculated
            self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)

        # 2) Checks if the robot should start turning but still be moving forward
        elif front_distance < self.start_turning:
            # Linear speed is calculated
            twist.linear.x = self.max_speed * self.speed_factor(front_distance)
            # Rotation direction is set
            self.set_rot_d(right_distances, left_distances) 
            # Rotation speed is calculated
            twist.angular.z = self.max_angle * self.angle_factor(front_distance) * self.turn_factor

        # 3 ) The robot should simply move forward with full speed    
        else:
            twist.linear.x = self.max_speed
            twist.angular.z = 0.
        
        # If the robot is currently detecting a color and it hasn't registered
        # a victim yet, it will slow down to detect the color more easily
        if self.color_detected and not self.victim_found:
            print("Slowing because color")
            if(twist.linear.x > self.max_speed * self.color_slow_factor):
                twist.linear.x = self.max_speed * self.color_slow_factor

        
        # If the robot is barely moving forward, the stuck counter is incremented
        if abs(twist.linear.x) < 0.01:
            self.stuck_counter +=1 
        else:
            self.stuck_counter = 0

        # Logging linear speed
        self.lin_speed = twist.linear.x
        
        # Accumulating the linear speed
        self.speed_acc += abs(twist.linear.x)
        # Incrementing the loop count
        self.loop_count += 1

        # Updating the robots movement
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()