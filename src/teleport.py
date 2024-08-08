#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess

def start_subprocess(command) -> None:
    process = subprocess.Popen(command, shell=True)
    print(f"\033[92m Teleoporting: {''.join(command)}\033[0m")
class State():
    def __init__(self, msg, name="vnymous", reference_frame="world"):
        self.name = name
        self.x = 10.0
        self.y = 10.0
        self.z = 0.0
        self.reference_frame = reference_frame

class Teleport(Node):
    
    def __init__(self):
        super().__init__("teleporter_node")
        self.subscription = self.create_subscription(Odometry, '/zed/zed_node/zed_odom', self.odom_callback, 10)
    def odom_callback(self, msg):
        odom = msg
        state = State(odom, name="vnymous", reference_frame="world")
        fstring = f'ros2 service call /gazebo/set_entity_state gazebo_msgs/SetEntityState "state: {{name: {state.name}, pose: {{position:{{x: {state.x}, y: {state.y}, z: {state.z}}}}}, reference_frame: {state.reference_frame}}}"'
        start_subprocess(fstring)
    
def main(args=None):
    rclpy.init(args=args)

    teleport = Teleport()

    rclpy.spin(teleport)

    teleport.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

