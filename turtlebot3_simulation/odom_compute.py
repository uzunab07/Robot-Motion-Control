#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from math import cos, sin, pi,atan2
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import JointState

import time

# Defining Manufacturer Constans
_WHEEL_CIRCUMFERENCE = 0.2073 # meters
_TICKS_PER_REVOLUTION = 5.7177
_RW = 0.1435 # meters


class State:
    def __init__(self,x,y,theta,vx,vy,omega):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.vy = vy
        self.vtheta = omega
    

class OdomCompute(Node):
    # Constructor
    def __init__(self):
        super().__init__("odom_compute")
        # Create Subscriber
        self.subscriber = self.create_subscription(JointState,'joint_states',self.subscriber_callback,10)
        
        # Create Publishers
        self.publisher_ = self.create_publisher(Odometry,'/my_odom',10)

        # Initialize variables
        self.flag_first_time = True
        self.prev_left_encoder_reading = 0
        self.prev_right_encoder_reading = 0
        self.state = State(0.0,0.0,0,0,0,0)
        
    def transition_model(self,x,u):
        """
        Predicts the new state of the robot using the current state (x),
        control inputs (u), and the elapsed time (dt).

        :param x: Current state (State object)
        :param u: Control input [left_wheel_change, right_wheel_change]
        :param dt: Time step
        :return: Updated state (State object)
        
        What I have left_wheel_change, right_wheel_change, _RW distance between the wheel, _WHEEL_CIRCUMFERENCE
        
        What I am computing the new state X = [x_new,y_new,theta_new, vx, vy, vtheta], Raduis of the curved path R
        
        """
        # Extracting the current state from the state x=[x,y,theta]
        x_old = x.x
        y_old = x.y
        theta_old = x.theta
        # Computing the dt which is the time elapsed since the last update
        dt = 1/ 30
        
        # # Extracting the control from u = [v,w] which is Twist object
        # v = u.linear.v
        # omega = u.angular.w
        
        # here instead of extracting directly the linear and angular velocity i have to compute them using dl and dr given in the control input
        # the formula is v = (vl + vr)/ 2 and w (omega)= vr - vl / 2 * _RW
        
        # First i will compute the change of angle Δθ, u[0] for left_wheel_change and u[1] right_wheel_change()
        
        left_wheel_change = u[0]
        right_wheel_change = u[1]
        
        Δθ = (right_wheel_change - left_wheel_change) / (2 * _RW)
        
        # Then the total distance travelled
        
        d = (right_wheel_change + left_wheel_change) / 2
        
       
        if abs(Δθ) > 1e-4:  # Raduis of the curved path R if Δθ  is not zero then the robot is moving in a circle
            R = (2 * _RW * left_wheel_change) / (right_wheel_change - left_wheel_change)
        
            # From this point I have calculated Δθ = θ_new - θ, R the raduis of the curved path and also the total distance travelled
            
            '''
            Now I can calculated the change the position to get thew new state (assuming the ICC is fixed) 
            The data I have to do that are the old state X = [x,y,θ], dr, dl
            Now i have to compute the new angle  θ_new = Δθ + θ
            
            
            '''
            
            # Computing the new angle θ_new
            θ_new = Δθ + theta_old
            
            # Normalize theta_new to the range [-pi, pi]
            # θ_new = (θ_new + pi) % (2 * pi) - pi
            
            θ_new = atan2(sin(θ_new),cos(θ_new))
            
            # Computing the ICC_x and ICC_y so i can calculate the change in position
            ICC_x = x_old - (R + _RW)*sin(theta_old)
            ICC_y = y_old + (R + _RW)*cos(theta_old)
            
            # Computing the position x_new and y_new in the case ICC is fixed
            x_new = ICC_x + (R + _RW)* sin(theta_old+Δθ)
            y_new = ICC_y - (R + _RW)* cos(theta_old+Δθ)
        else:  # If Δθ  is  close zero then the robot is moving straight
            x_new = x_old + d* cos(theta_old)
            y_new = y_old + d* sin(theta_old)
            
            θ_new = theta_old  +Δθ# Orientation remains unchanged
        
        
        # Computing the linear and angular velocity during travel from x to x_new and y_to_y_new 
        vx = (x_new - x_old)/ dt
        vy = (y_new - y_old) / dt
        omega = Δθ / dt

        
            
            
         # Assigning the new state to the state object
        x.x = x_new
        x.y = y_new
        x.theta = θ_new
        x.vx = vx
        x.vy = vy
        x.vtheta = omega
        
        return x
      
    def  subscriber_callback(self,data): #encoder_callback
        left_encoder_index = data.name.index('wheel_left_joint')
        right_encoder_index = data.name.index('wheel_right_joint')
        
        
        if self.flag_first_time:
            self.prev_left_encoder_reading = data.position[left_encoder_index]
            self.prev_right_encoder_reading = data.position[right_encoder_index]
            self.flag_first_time = False
        else:
            
            left_encoder = data.position[left_encoder_index]
            right_encoder = data.position[right_encoder_index]
            
            # Computing distance travelled by both wheels
            
            dl  = (left_encoder - self.prev_left_encoder_reading) * _WHEEL_CIRCUMFERENCE / _TICKS_PER_REVOLUTION 
            dr = (right_encoder - self.prev_right_encoder_reading) * _WHEEL_CIRCUMFERENCE / _TICKS_PER_REVOLUTION 
            
            # Updating the previous encoder readings
            self.prev_left_encoder_reading = left_encoder
            self.prev_right_encoder_reading = right_encoder

            # Creating the control 
            u = [dl,dr]
            
            self.state = self.transition_model(self.state,u)
        
            odom = Odometry()
            odom.pose.pose.position.x = self.state.x
            odom.pose.pose.position.y = self.state.y
            odom.pose.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, self.state.theta)
            odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            odom.twist.twist.linear.x = float(self.state.vx)
            odom.twist.twist.linear.y = float(self.state.vy)
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = float(self.state.vtheta)
    
            
            
            # Publishing a nav_msgs/Odometry
            
            
            self.publisher_.publish(odom)
            # self.get_logger().info(f'Message Published {my_odom_publisher_}')
        
        
def main (args = None):
    rclpy.init(args=args)
    node = OdomCompute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
        
        
        
        
        
        
        
        
        
        
