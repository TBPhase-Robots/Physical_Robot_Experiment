
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32

import random
import string
from geometry_msgs.msg import Pose, Vector3

from std_msgs.msg import Float64, Int32, String

from std_msgs.msg import ColorRGBA

class SimulationNode(Node):

    def __init__(self):

        
        rId = random.randint(0, 99999)
        pubName = f'SimulationNode{rId}'
        
        super().__init__(pubName)
        print("created node ", pubName)
    
    
    def CreateAgentListener(self, topic_name, controller_callback):
        print("created subscription: " , topic_name)
        sub = self.create_subscription(Int32, topic_name, controller_callback, 10)
        return sub

    def CreateStringListener(self, topic_name, controller_callback):
        print("created subscription: " , topic_name)
        sub = self.create_subscription(String, topic_name, controller_callback, 10)
        return sub


    #   topic_name = f'/robot{id}/pose'
    def CreatePoseListener(self, topic_name, controller_callback):

        print("created subscription: " , topic_name)
        sub = self.create_subscription(Pose, topic_name, controller_callback, 10)
        return sub


    # =========PUBLISHERS============#

    def CreateFloatPublisher(self, topic_name):
        print("created publisher: " , topic_name)
        pub = self.create_publisher(Float64, topic_name, 10)
        return pub

    def CreateIntPublisher(self, topic_name):
        print("created publisher: " , topic_name)
        pub = self.create_publisher(Int32, topic_name, 10)
        return pub

    def CreateVectorPublisher(self, topic_name):
        print("created publisher: " , topic_name)
        pub = self.create_publisher(Vector3, topic_name, 10)
        return pub
    
    def CreateColourPublisher(self, topic_name):
        print("created publisher: " , topic_name)
        pub = self.create_publisher(ColorRGBA, topic_name, 10)
        return pub
      