#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor

class dist_publisher(Node):
    def __init__(self, node_name="dist_publisher"):
        self.keep_dist = 1.0 #distance in m to keep from lead car
        
        # PID tuning
        self.p_a = 1.5 #P term for turning
        self.p_d = 1.5 #P term for throttle
        self.d_a = 0.0 #D term for turning
        self.d_d = 0.2 #D term for throttle
        
        #for d term
        self.last_a_err = 0
        self.last_d_err = 0
        
        
        self._node_name = node_name
        self.data = []
        self.tar_ang = math.pi
        self.tar_ind = round(math.pi/2/0.014005)
        self.ang_inc = 0.2
        self.tar_found = False
        super().__init__(self._node_name)

        #subscriber to get lidar data
        self.subscriber_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        #subscriber to get camera data
        self.subscriber_cam = self.create_subscription(
            Float32MultiArray,
            '/camera_data',
            self.target_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publish_dist)

        self.get_logger().info(self._node_name +" Ready...")

    # callback for lidar subscriber
    def laserscan_callback(self, msg):
        # Save the data for the node to use later
        self.data = msg.ranges
        self.ang_inc = msg.angle_increment

    # callback for camera target subscriber
    def target_callback(self,msg):
    	if msg.data[1] == float('inf'):
    		self.tar_found = False
    	else:
    		self.tar_found = True
    		self.tar_ang = math.radians(msg.data[1]) + math.pi/2
    		self.tar_ind = round(self.tar_ang/self.ang_inc)

    def publish_dist(self):
        msg = Twist()
        if len(self.data) == 0 or self.tar_found == False or self.data[self.tar_ind] == float('inf'):
            #self.get_logger().info('No data')
            pass
        else:
        	dist = self.data[self.tar_ind]
        	err = dist - self.keep_dist
        	temp = self.p_d * err + self.d_d * (err - self.last_d_err)
        	msg.linear.x = min(1.0, temp)
        	#msg.linear.x = temp
        	#self.get_logger().info('linear: "%f"' % msg.linear.x)
        	#msg.linear.x = 0.5
        	self.last_d_err = err
        	ang_err = math.pi/2 - self.tar_ang
        	msg.angular.z = self.p_a * ang_err + self.d_a * (ang_err - self.last_a_err)
        	if msg.linear.x < 0:
        		msg.angular.z = -msg.angular.z
        	self.last_a_err = ang_err
        	#self.get_logger().info('Distance to target: "%f"' % dist)
        	#self.get_logger().info('Using angle: "%f"' % math.degrees(ang_err))
        	
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = dist_publisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        # Spin the executor
        executor.spin()
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

