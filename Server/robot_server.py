
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

class Robot():
    id: int
    marker_publisher: Publisher
    heartbeat_subscription: Subscription
    alive: bool = True
    time_since_heartbeat: float = 0

    def heartbeat_callback(self, msg: Bool):
        self.alive = True
        self.time_since_heartbeat = 0

class Server(Node):
    next_id: int = 0
    active_robot_ids: List[int] = []
    active_robots: List[Robot] = []

    death_time: float = 20

    ids_publisher: Publisher = None
    added_robot_publisher: Publisher = None
    removed_robot_publisher: Publisher = None

    register_subscription: Subscription = None

    def __init__(self):
        super().__init__("server")

        self.ids_publisher = self.create_publisher(Int32, '/setup/ids', 10)
        self.register_subscription = self.create_subscription(Int32, '/setup/register', self.register_callback, 10)
        self.added_robot_publisher = self.create_publisher(Int32, '/global/robots/added', 10)
        self.removed_robot_publisher = self.create_publisher(Int32, '/global/robots/removed', 10)

    def setup_robot(self, robot: Robot):
        robot.marker_publisher = self.create_publisher(Int64, f'/robot{robot.id}/markers', 10)
        robot.heartbeat_subscription = self.create_subscription(Bool, f'/robot{robot.id}/heartbeat', robot.heartbeat_callback, 10)
        robot.time_since_heartbeat = 0

        marker = marker_maker.marker_to_int(robot.id)
        time.sleep(3)

        marker_msg = Int64()
        marker_msg.data = marker
        print('publishing marker')
        robot.marker_publisher.publish(marker_msg)

    def register_callback(self, msg: Int32):
        if msg.data == -1:
            self.active_robot_ids.append(self.next_id)
            robot = Robot()
            robot.id = self.next_id
            self.active_robots.append(robot)

            print(f'Sending id: {self.next_id}')

            id_msg = Int32()
            id_msg.data = self.next_id
            self.ids_publisher.publish(id_msg)

            self.next_id += 1
        else:
            id_count = self.active_robot_ids.count(msg.data)
            if id_count > 1:
                print(f'Duplicate id: {msg.data}')
            else:
                robot_index = self.active_robot_ids.index(msg.data)
                robot = self.active_robots[robot_index]
                self.setup_robot(robot)

                id_msg = Int32()
                id_msg.data = robot.id
                print(f'Publishing new robot')
                self.added_robot_publisher.publish(id_msg)

    def check_alive(self, time_delta):
        for robot in self.active_robots:
            if robot.alive:
                robot.time_since_heartbeat += time_delta

                if robot.time_since_heartbeat > self.death_time * 1000 * 1000 * 1000:
                    self.kill_robot(robot)

    def kill_robot(self, robot: Robot):
        id_msg = Int32()
        id_msg.data = robot.id
        self.removed_robot_publisher.publish(id_msg)

        self.destroy_subscription(robot.heartbeat_subscription)
        self.destroy_publisher(robot.marker_publisher)


def main():
    rclpy.init()

    server = Server()
    past_time = time.time_ns()

    while rclpy.ok():
        rclpy.spin_once(server, timeout_sec=200)
        current_time = time.time_ns()
        # server.check_alive(current_time - past_time)
        past_time = current_time


    rclpy.shutdown()

if __name__ == "__main__":
    main()
