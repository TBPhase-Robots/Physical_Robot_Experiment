
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32
import random

# DEPRECATED FILE, SIMULATION NODE IS NOW USED
class IntPublisher(Node):

    def __init__(self, topic_name):
        rId = random.randint(0, 999999)
        pubName = f'intPublisher{rId}'
        super().__init__(pubName)
        print("created publisher: " , topic_name)
        self.pub = self.create_publisher(Int32, topic_name, 10)
      
        


    def main(args=None):
        

        node = IntPublisher()
        


