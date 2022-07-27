
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose

class Listener(Node):

    def __init__(self, id, controller_callback):
        
        super().__init__(f'listener{id}')

        topic_name = f'/robot{id}/pose'
        print("created subscription: " , topic_name)
        self.sub = self.create_subscription(Pose, topic_name, controller_callback, 10)
      

    # controller callback sends the update to the simulation controller to go to the next event state
    def chatter_callback(self, msg):
        print(msg)
        self.get_logger().info('I heard: [%s]' % msg.data)
        


    def main(args=None):
        

        node = Listener()
        


