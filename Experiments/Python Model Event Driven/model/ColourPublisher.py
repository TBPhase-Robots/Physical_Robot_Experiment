
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import ColorRGBA


class ColourPublisher(Node):

    def __init__(self, topic_name, robotId):
        
        

        rId = str(robotId)
        pubName = f'robot{rId}_colour_publisher'
        super().__init__(pubName)
        print("created publisher: " , topic_name)
        self.pub = self.create_publisher(ColorRGBA, topic_name, 10)
      


    def main(args=None):
        

        node = ColourPublisher("aaaa", 99)
        


