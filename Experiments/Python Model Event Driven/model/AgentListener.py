
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
import random
import string

# DEPRECATED FILE, SIMULATION NODE IS NOW USED
class AgentListener(Node):

    def __init__(self, topic_name, controller_callback):
        #letters = string.ascii_lowercase

        rId = random.randint(0, 9999)
        pubName = f'AgentListener{rId}'
        
        super().__init__(pubName)
        print("created subscription: " , topic_name)
        self.sub = self.create_subscription(Int32, topic_name, controller_callback, 10)
        


        


    def main(args=None):
        

        node = AgentListener()
        


