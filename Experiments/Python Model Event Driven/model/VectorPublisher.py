
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class VectorPublisher(Node):

    def __init__(self, topic_name, robotId):
        
#        pubName = "robot", str(robotId)
        rId = str(robotId)
        pubName = f'robot{rId}_vector_publisher'
        super().__init__(pubName)
        print("created publisher: " , topic_name)
        self.pub = self.create_publisher(Vector3, topic_name, 10)
      
        


    def main(args=None):
        

        node = VectorPublisher()
        


