#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import time
def start_subprocess(command) -> None:

    process = subprocess.Popen(command, shell=True)
    print(f"\033[92m Teleoporting: {''.join(command)}\033[0m")
class Orientation():
    def __init__(self, msg):
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w

        
class State():
    def __init__(self, msg, name="vnymous", reference_frame="world"):
        self.name = name
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = 0.0
        self.orientation = Orientation(msg)
        self.reference_frame = reference_frame

class Teleport(Node):
    
    def __init__(self):
        super().__init__("teleporter_node")
        self.subscription = self.create_subscription(Odometry, '/zed/zed_node/transformed_odom', self.odom_callback, 10)
    def odom_callback(self, msg):
        odom = msg
        state = State(odom, name="vnymous", reference_frame="world")
        fstring = f'ros2 service call /gazebo/set_entity_state gazebo_msgs/SetEntityState "state: {{name: {state.name}, pose: {{position:{{x: {state.x}, y: {state.y}, z: {state.z}}}, orientation: {{x: {state.orientation.x}, y: {state.orientation.y}, z: {state.orientation.z}, w: {state.orientation.w}}}}}, reference_frame: {state.reference_frame}}}"'
        start_subprocess(fstring)
        time.sleep(1)
    
def main(args=None):
    rclpy.init(args=args)

    teleport = Teleport()

    rclpy.spin(teleport)

    teleport.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

