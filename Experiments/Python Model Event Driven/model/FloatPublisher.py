
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Float64


# DEPRECATED FILE, SIMULATION NODE IS NOW USED

class FloatPublisher(Node):

    def __init__(self, topic_name, robotId):
        
        

        rId = str(robotId)
        pubName = f'robot{rId}_float_publisher'
        super().__init__(pubName)
        print("created publisher: " , topic_name)
        self.pub = self.create_publisher(Float64, topic_name, 10)
      


    def main(args=None):
        

        node = FloatPublisher("aaaa", 99)
        


