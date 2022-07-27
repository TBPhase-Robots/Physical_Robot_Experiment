
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class Publisher(Node):

    def __init__(self, topic_name):
        
        super().__init__('talker')
        print("created publisher: " , topic_name)
        self.pub = self.create_publisher(String, topic_name, 10)
      
        


    def main(args=None):
        

        node = Publisher()
        


