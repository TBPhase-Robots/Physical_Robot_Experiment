
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose

class CommandListener(Node):

    def __init__(self, topic_name, controller_callback):
        
        super().__init__('listener')
        print("created subscription: " , topic_name)
        self.sub = self.create_subscription(String, topic_name, controller_callback, 10)
      

    # controller callback sends the update to the simulation controller to go to the next event state
    def chatter_callback(self, msg):
        print(msg)
        self.get_logger().info('I heard: [%s]' % msg.data)
        


    def main(args=None):
        

        node = CommandListener()
        


