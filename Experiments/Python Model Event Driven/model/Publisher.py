
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
import random

class Publisher(Node):

    def __init__(self, topic_name):
        rId = random.randint(0, 9999)
        pubName = f'stringPublisher{rId}'
        super().__init__(pubName)
        print("created publisher: " , topic_name)
        self.pub = self.create_publisher(String, topic_name, 10)
      
        


    def main(args=None):
        

        node = Publisher()
        


