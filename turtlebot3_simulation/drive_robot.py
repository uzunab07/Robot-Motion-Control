import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from math import sqrt,pow
import numpy as np
MAX_SPEED = 5.0
MIN_SPEED = 0.01

class TurtleBotControl(Node):
    
    def __init__(self):
        super().__init__('drive_robot_KM')
        
        # KP proportional gain
        self.Kp = 0.5
        
        self.initial_odom = Odometry()
        self.current_odom = Odometry()

        # Creating a subscriber to receive odometry data from my_odom topic
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.subscriber_ # prevent unsed variable warning
        
        
        # Creating a publisher to publish Twist commands to cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # creating a timer to publish Twist
        self.timer_ = self.create_timer(1.0, lambda: self.drive_distance(1))
        
 
        
        # Flags to track driving and turning status
        self.has_turned = False
        
    def drive_distance(self, goal_distance):
        cmd_vel = Twist()
        # calculate the distance covered by the robot
        distance_covered =  sqrt(pow(self.current_odom.pose.pose.position.x - self.initial_odom.pose.pose.position.x, 2) + pow(self.current_odom.pose.pose.position.y - self.initial_odom.pose.pose.position.y, 2))
     
        error  = float(goal_distance*(np.sign(goal_distance)) - distance_covered)
        
        if np.abs(error) < 0.01:
             self.publisher_.publish(cmd_vel)
             return
        else:    
           cmd_vel.linear.x = np.sign(goal_distance) * max(min(self.Kp * np.abs(error),MAX_SPEED),MIN_SPEED)
        self.publisher_.publish(cmd_vel)
        self.get_logger().info(f"Error: {error}")
        
    def turn_to(self,angle_rads, current_odom):
        # geeting the orientation (theta) by converting quaternion to euler angles
        orientation_q = current_odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # converting euler angles to rads
        (_,_,theta) = euler_from_quaternion(orientation_list)  # Only need yaw (theta)
        
 
        # calculate the difference in angle between the current orientation and the desired angle
        angle_diff_rads = angle_rads - theta
        
        if abs(angle_diff_rads) > 0.01: # if the angle difference is greater than 0.01 rads then turn the robot
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.2 if angle_diff_rads > 0 else -0.2
            self.publisher_.publish(self.cmd_vel)
        else: # if the angle difference is less than 0.1 we stop turning the robot
            self.cmd_vel.linear.x= 0.0
            self.cmd_vel.angular.z = 0.0
            self.publisher_.publish(self.cmd_vel)
            self.has_turned = True
            self.get_logger().info(f"Turn completed to {angle_rads} radians.")

 
    
    # Creating a callback function to handle odometry data
    
    def odom_callback(self,data):
   
        # print out the data in the callback function
        # 1. Get the position (x, y)
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
    
        #2. get the orientation (theta) by converting quaternion to euler angles
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, theta) = euler_from_quaternion(orientation_list)  # Only need yaw (theta)
        
        #3. get the angular and linear velocity (vx, vtheta)
        vx = data.twist.twist.linear.x
        vtheta = data.twist.twist.angular.z

        # setting the initial odometry if this is the first time
        if self.initial_odom is None:
             self.initial_odom = data
        self.current_odom = data     
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()