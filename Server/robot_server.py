
import sys
sys.path.append('..')

import time
from typing import List
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Int64

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import marker_maker

from Logging.ros_logger import RosLogger

# Class to store data about a robot
class Robot():
    id: int
    marker_publisher: Publisher
    heartbeat_subscription: Subscription

    alive: bool = False
    time_since_heartbeat: float = 0

    # Callback function, called when the robots send heartbeat messages to
    # indicate that they are still live
    def heartbeat_callback(self, msg: Bool):
        self.alive = True
        self.time_since_heartbeat = 0

class Server(Node):
    next_id: int = 0
    active_robot_ids: List[int] = []
    active_robots: List[Robot] = []

    # Time taken for a robot that sends no hearbeats to be removed from the system
    death_time: float = 20

    ids_publisher: Publisher = None
    added_robot_publisher: Publisher = None
    removed_robot_publisher: Publisher = None

    register_subscription: Subscription = None

    def __init__(self):
        # Create ROS node
        super().__init__("server")

        # Setup logging system
        self.logger = RosLogger(self, 'server')

        # Create publishers
        self.ids_publisher = self.create_publisher(Int32, '/setup/ids', 10)
        self.added_robot_publisher = self.create_publisher(Int32, '/global/robots/added', 10)
        self.removed_robot_publisher = self.create_publisher(Int32, '/global/robots/removed', 10)

        # Create subscriptions
        self.register_subscription = self.create_subscription(Int32, '/setup/register', self.register_callback, 10)

    # Sends a marker to the robot and sets up a heartbeat checker for it
    def setup_robot(self, robot: Robot):
        robot.marker_publisher = self.create_publisher(Int64, f'/robot{robot.id}/markers', 10)
        robot.heartbeat_subscription = self.create_subscription(Bool, f'/robot{robot.id}/heartbeat', robot.heartbeat_callback, 10)

        robot.time_since_heartbeat = 0

        # Generates a marker for the robot
        marker = marker_maker.marker_to_int(robot.id)

        # Waits because MicroROS is slow and tempermental
        time.sleep(3)

        # Sends a marker to the robot
        marker_msg = Int64()
        marker_msg.data = marker
        self.logger.log(f'publishing marker for robot {robot.id}')
        robot.marker_publisher.publish(marker_msg)

    # Called when a robot requests an id or acknowledges recieving an id
    def register_callback(self, msg: Int32):
        # If -1 is recieved, a robot is requesting an id
        if msg.data == -1:
            # Add the new robot to the list of active robots
            self.active_robot_ids.append(self.next_id)
            robot = Robot()
            robot.id = self.next_id
            self.active_robots.append(robot)

            # Send the new robot an id
            self.logger.log(f'Sending id: {self.next_id}')
            id_msg = Int32()
            id_msg.data = self.next_id
            self.ids_publisher.publish(id_msg)

            # Increment the id, so that the next robot to request an id 
            # recieves a unique id
            self.next_id += 1

        # Otherwise an id is being acknowldeged
        else:
            # Check for duplicate ids
            id_count = self.active_robot_ids.count(msg.data)
            if id_count > 1:
                self.logger.warn(f'Duplicate id: {msg.data}')

            else:
                # find the robot data for the robot that is acknowleding
                # the id
                robot_index = self.active_robot_ids.index(msg.data)
                robot = self.active_robots[robot_index]

                self.setup_robot(robot)

                # Announce the new robot to the rest of the system
                id_msg = Int32()
                id_msg.data = robot.id
                self.logger.log(f'Publishing new robot')
                self.added_robot_publisher.publish(id_msg)

    # Checks that the robots are still publishing heartbeats
    def check_alive(self, time_delta):
        for robot in self.active_robots:

            # Checks that the robot has been set up
            if robot.alive:

                # Calculate the new time since last heartbeat
                robot.time_since_heartbeat += time_delta

                # Kill the robot if it hasn't sent a new heartbeat in time
                if robot.time_since_heartbeat > self.death_time * 1000 * 1000 * 1000:
                    self.kill_robot(robot)

    # Removes a robot from the system
    def kill_robot(self, robot: Robot):
        # Announce that the robot is being removed
        id_msg = Int32()
        id_msg.data = robot.id
        self.removed_robot_publisher.publish(id_msg)

        # Remove the robots data from the server
        self.destroy_subscription(robot.heartbeat_subscription)
        self.destroy_publisher(robot.marker_publisher)
        self.active_robot_ids.remove(robot.id)
        self.active_robots.remove(robot)


def main():
    # Initialise ROS
    rclpy.init()

    # Create the server
    server = Server()

    # Record the time
    past_time = time.time_ns()

    while rclpy.ok():
        # Check for new ROS messages
        rclpy.spin_once(server, timeout_sec=0.2)

        # Update the current time and check if the robots are alive
        current_time = time.time_ns()
        server.check_alive(current_time - past_time)
        past_time = current_time


    rclpy.shutdown()

if __name__ == "__main__":
    main()
