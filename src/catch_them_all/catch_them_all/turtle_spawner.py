#!/usr/bin/env python3


from rclpy.node import Node
import rclpy
from turtlesim.srv import Spawn,Kill
import random
from functools import partial
from turtle_interfaces.msg import Turtle, TurtleArray
from turtle_interfaces.srv import CatchTurtle


class TurtleSpawner (Node):
    def __init__ (self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawner_frequency", 1.0)

        self.spawner_frequency = self.get_parameter("spawner_frequency").value

        self.turtle_name_prefix_ = "turtle"
        self.turtle_counter = 2

        self.turtle_request_ = self.create_client(Spawn,"spawn")
        self.turtle_kill_request_ = self.create_client(Kill,"kill")
        self.spawner_timmer = self.create_timer(self.spawner_frequency,self.new_turtle_spawner)
        
        self.turtle_list = TurtleArray()
        self.turtle_list.turtles = []
        self.alive_turtle_publisher_ = self.create_publisher(TurtleArray,"alive_turtles",10)

        self.kill_server_ = self.create_service(CatchTurtle,"catch_turtle",self.catch_turtle_callback)


    def catch_turtle_callback(self, request:CatchTurtle.Request, response:CatchTurtle.Response):

        for i in self.turtle_list.turtles:
            if i.name == request.name:
                msg = Kill.Request()
                msg.name = i.name
                self.turtle_kill_request_.call_async(msg)

                self.turtle_list.turtles.remove(i)
                self.alive_turtle_publisher_.publish(self.turtle_list)
        return response


                


    def new_turtle_spawner(self):
        self.turtle_name_ = f"{self.turtle_name_prefix_}{self.turtle_counter}"
        x = random.uniform(1.0, 10.0) 
        y = random.uniform(1.0, 10.0) 
        theta = random.uniform(0.0,360.0)
        self.turtle_counter += 1
        self.spawner_callback(x,y,theta,self.turtle_name_)



    def spawner_callback(self,x,y,theta, name):
        while not self.turtle_request_.wait_for_service(1.0):
            self.get_logger().info("waiting for service ready.")

        turtle_service_ = Spawn.Request()
        turtle_service_.x = x
        turtle_service_.y = y
        turtle_service_.theta = theta
        turtle_service_.name = name
        future = self.turtle_request_.call_async(turtle_service_)
        future.add_done_callback(partial(self.callback_call_spawner, request = turtle_service_))


    def callback_call_spawner(self,future,request:Spawn.Request):
        response: Spawn.Response = future.result()
        if response != "":
            self.get_logger().info(f"{response} has been generated...")
            self.new_turtle = Turtle()
            self.new_turtle.x = request.x
            self.new_turtle.y = request.y
            self.new_turtle.theta = request.theta
            self.new_turtle.name = response.name
            self.turtle_list.turtles.append(self.new_turtle)
            self.alive_turtle_publisher_.publish(self.turtle_list)

 

def main(args = None):
    
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        

        



