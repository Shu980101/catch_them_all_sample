#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi
from turtle_interfaces.msg import Turtle, TurtleArray
from turtle_interfaces.srv import CatchTurtle


class TurtleController(Node):
    def __init__ (self):
        super().__init__("turtle_controller")
        self.declare_parameter("linear_speed",2.0)
        self.declare_parameter("angular_speed",2.5)
        self.declare_parameter("control_frequency",0.1)
        self.declare_parameter("closest_turtle_to_catch_first", True)


        self.linear_speed = self.get_parameter("linear_speed").value
        self.angluar_speed = self.get_parameter("angular_speed").value
        self.control_frequency = self.get_parameter("control_frequency").value
        self.closest_turtle_to_catch_first = self.get_parameter("closest_turtle_to_catch_first").value


        self.pose_: Pose = None
        self.first_turtle_to_catch:Turtle = None
        
        self.catch_client = self.create_client(CatchTurtle,"catch_turtle")
        self.vel_publicher =self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pose_suscriber = self.create_subscription(
            Pose,"/turtle1/pose",self.callback_pose_suscriber, 10)
        self.alive_turtle_suscriber = self.create_subscription(TurtleArray,"/alive_turtles",self.alive_turtle_callback,10)
        

        self.timer_ = self.create_timer(self.control_frequency,self.control_loop)


    def alive_turtle_callback (self, msg:TurtleArray):
        if len(msg.turtles) > 0:
            if self.closest_turtle_to_catch_first:
                closest_turtle = None
                closest_dist = None

                for turtle in msg.turtles:
                    dist_x = turtle.x  - self.pose_.x
                    dist_y = turtle.y  - self.pose_.y
                    dist = sqrt(dist_x*dist_x + dist_y*dist_y)

                    if closest_dist == None or dist<closest_dist:
                        closest_turtle = turtle
                        closest_dist = dist

                self.first_turtle_to_catch = closest_turtle
                        
                
            else:
                self.first_turtle_to_catch = msg.turtles[0]
        
    def callback_pose_suscriber(self,msg:Pose):
        self.pose_=msg

    def control_loop (self):
        if self.pose_ == None or self.first_turtle_to_catch == None:
            return
        
        goal_pose = self.first_turtle_to_catch
        dist_x = goal_pose.x  - self.pose_.x
        dist_y = goal_pose.y  - self.pose_.y
        dist = sqrt(dist_x*dist_x + dist_y*dist_y)

        cmd = Twist()
        if dist > 0.5:

            #posotion
            cmd.linear.x = dist*self.linear_speed

            #angular
            goal_theta = atan2(dist_y,dist_x)
            diff = goal_theta - self.pose_.theta
            
            if diff > pi:
                diff -= 2*pi
            elif diff <-pi:
                diff+= 2*pi
            cmd.angular.z = self.angluar_speed*diff
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            msg = CatchTurtle.Request()
            msg.name = self.first_turtle_to_catch.name
            self.catch_client.call_async(msg)
            self.get_logger().info("Goal reached. Stopping the turtle.")

        self.vel_publicher.publish(cmd)


def main(args=None):
    
    rclpy.init(args=args)

    node = TurtleController()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()