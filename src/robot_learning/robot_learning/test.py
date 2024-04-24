#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.qos import ReliabilityPolicy, HistoryPolicy
import time


class robot_control(Node):
    def __init__(self):
        super().__init__('reading_laser')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',  # Change topic name accordingly
            self.info,
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
        )

    def info(self, data):
        # regions = {
        #     'right':  min(msg.ranges[55:90]),
        #     'fright': min(msg.ranges[19:54]),
        #     'front':  min(msg.ranges[0:18] + msg.ranges[342:360]),
        #     'fleft':  min(msg.ranges[306:341]),
        #     'left':   min(msg.ranges[270:305]),
        # }
        # regions = {
        #     'right':  min(msg.ranges[45:90]),
        #     # 'fright': min(msg.ranges[19:54]),
        #     'front':  min(msg.ranges[0:44] + msg.ranges[315:360]),
        #     # 'fleft':  min(msg.ranges[306:341]),
        #     'left':   min(msg.ranges[270:314]),
        # }
        collision_region = {
            'right':  min(data.ranges[55:90]),
            'fright': min(data.ranges[19:27]),
            'front':  min(data.ranges[0:18] + data.ranges[342:360]),
            'fleft':  min(data.ranges[335:341]),
            'left':   min(data.ranges[270:305]),
        }


        self.take_action(collision_region)


    def take_action(self, collision_region):
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0
        
        state_description = ''
        
        if collision_region['front'] > 0.7 and collision_region['fleft'] > 0.7 and collision_region['fright'] > 0.7:
            state_description = 'case 1 - nothing'
            linear_x = 0.6
            angular_z = 0.0

        elif collision_region['front'] < 0.7 and collision_region['fleft'] > 0.7 and collision_region['fright'] > 0.7:
            state_description = 'case 2 - front'
            linear_x = 0.0
            angular_z = -0.3

        elif collision_region['front'] > 0.7 and collision_region['fleft'] > 0.7 and collision_region['fright'] < 0.7:
            state_description = 'case 3 - fright'
            linear_x = 0.0
            angular_z = -0.3
        
        elif collision_region['front'] > 0.7 and collision_region['fleft'] < 0.7 and collision_region['fright'] > 0.7:
            state_description = 'case 4 - fleft'
            linear_x = 0.0
            angular_z = -0.3
      
        elif collision_region['front'] < 0.7 and collision_region['fleft'] > 0.7 and collision_region['fright'] < 0.7:
            state_description = 'case 5 - front and fright'
            linear_x = 0.0
            angular_z = -0.3

        elif collision_region['front'] < 0.7 and collision_region['fleft'] < 0.7 and collision_region['fright'] > 0.7:
            state_description = 'case 6 - front and fleft'
            linear_x = 0.0
            angular_z = -0.3

        elif collision_region['front'] < 0.7 and collision_region['fleft'] < 0.7 and collision_region['fright'] < 0.7:
            state_description = 'case 7 - front and fleft and fright'
            linear_x = 0.0
            angular_z = -0.3

        elif collision_region['front'] > 0.7 and collision_region['fleft'] < 0.7 and collision_region['fright'] < 0.7:
            state_description = 'case 8 - fleft and fright'
            linear_x = 0.3
            angular_z = 0.0

        else:
            state_description = 'unknown case'
            self.get_logger().info(collision_region)

        self.get_logger().info(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)
        if state_description == 'case 2 - front' or state_description == 'case 3 - fright' or state_description == 'case 4 - fleft' or state_description == 'case 5 - front and fright' or state_description == 'case 6 - front and fleft' or state_description == 'case 7 - front and fleft and fright' :
            time.sleep(0.1)
def main(args=None):
    rclpy.init(args=args)
    laser_subscriber = robot_control()
    rclpy.spin(laser_subscriber)
    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
