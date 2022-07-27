
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose

class AgentListener(Node):

    def __init__(self, topic_name, controller_callback):
        
        super().__init__('agent_listener')
        print("created subscription: " , topic_name)
        self.sub = self.create_subscription(Int32, topic_name, controller_callback, 10)
      


        


    def main(args=None):
        

        node = AgentListener()
        


