import math
import sys
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose

class Talker(Node):

    def __init__(self):
        super().__init__('talker')

        
        self.i = 0
        self.pub = self.create_publisher(Pose, '/robot0/poses', 10)
        timer_period = 0.1
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        pose = Pose()

        randx = random.uniform(0,500)
        randy = random.uniform(0,500)
        randy = random.uniform(0,500)
        pose.position.x = randx
        pose.position.y = randy
        pose.orientation.z = random.uniform(0,2*math.pi)


        

        self.get_logger().info('Publishing: "{0}"'.format(pose))

        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()