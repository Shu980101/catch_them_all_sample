#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
import rclpy.node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.goal_pose = Pose()
        self.goal_pose.x = 2.0
        self.goal_pose.y = 8.0


        self.pose_ = None

        self.pose_suscriber_ = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)

        self.publisher_timmer = self.create_timer(0.1,self.control_loop)

    def pose_callback(self,msg:Pose):
        self.pose_ = msg

    def control_loop(self):
        if self.pose_ == None:
            return
        dist = self.distance(self.pose_,self.goal_pose) 
        angle = self.rotation(self.pose_,self.goal_pose)

        msg = Twist()
        if dist > 0.5:
            msg.linear.x = dist *2
            diff = angle - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff <-math.pi:
                diff+= 2*math.pi
            msg.angular.z = 5*diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)

    def distance(self,pose:Pose,goal_pose:Pose):
        x_diff = goal_pose.x - pose.x
        y_diff = goal_pose.y - pose.y
        
        distance =math.sqrt(x_diff*x_diff + y_diff*y_diff)

        return distance
    
    def rotation(self,pose:Pose,goal_pose:Pose):
        x_diff = goal_pose.x - pose.x
        y_diff = goal_pose.y - pose.y
        angle = math.atan2(y_diff , x_diff)

        return angle
        
def main(args = None):

    rclpy.init(args=args)

    node = TurtleController()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()