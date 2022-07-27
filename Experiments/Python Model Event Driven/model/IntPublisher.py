
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32


class IntPublisher(Node):

    def __init__(self, topic_name):
        
        super().__init__('intPublisher')
        print("created publisher: " , topic_name)
        self.pub = self.create_publisher(Int32, topic_name, 10)
      
        


    def main(args=None):
        

        node = IntPublisher()
        


