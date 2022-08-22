

"""Robot server is responsible for the setup and assignment of IDs to the 3Pi+ robots.

It is not responsible for their behaviour.

The robot server maintains a heartbeat ping with all robots to determine their life status.

Click on an item in the navigation subtree for detailed method documentation.

  """


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

class Robot():
    id: int
    marker_publisher: Publisher
    heartbeat_subscription: Subscription
    alive: bool = False
    time_since_heartbeat: float = 0


   

    def heartbeat_callback(self, msg: Bool):
        """ Heartbeat_callback is a function used to facilitate a bouncing ping between robot and server."""
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


        """ Initialisation function called upon instantiation of the the server. 
        The server is of type ROS Node and inherits its behaviours.
        Node superclass initialisation is called with name 'server', creating a ROS node with the name 'server'.
        
        super().__init__("server")

        ROS publisher and subscriber topics are created on the server node:

        Subscribers:
            
            Robot Registration Subscriber:

            Listens to the /setup/register ROS topic. Messages are published from the StackControl.ino script via
            the m5 stack aboard each robot. When each robot has successfully finished its setup routine, the message is published
            and self.register_callback function ( register_callback(self, msg: Int32) ) called.

            create_subscription(Int32, '/setup/register', self.register_callback, 10)

        Publishers:

            Agent ID Publisher:

            Once a robot object has been instantiated and successfully registered via register_callback,
            robot agent IDs are published to this topic.

            create_publisher(Int32, '/setup/ids', 10)



            Added Robot Publisher:

            At the end of the process of setting up a new robot, IDs are sent to the agent simulation via the '/global/robots/added'
            topic. These messages are used to add a new agent with specified ID to the simulation (runSimulation process). The robot
            with the specified ID may now be sent movement commands by simulation command and control.


            create_publisher(Int32, '/global/robots/added', 10)


            Removed Robot Publisher:

            Agents may be removed from simulation command and control by specified ID via the /global/robots/removed topic.
            Removed robots will no longer have an agent in the simulation to dictate their movements, and as such are inactive until
            re-added.
            create_publisher(Int32, '/global/robots/removed', 10)


        """


        super().__init__("server")

        self.logger = RosLogger(self, 'server')

        self.ids_publisher = self.create_publisher(Int32, '/setup/ids', 10)
        self.register_subscription = self.create_subscription(Int32, '/setup/register', self.register_callback, 10)
        self.added_robot_publisher = self.create_publisher(Int32, '/global/robots/added', 10)
        self.removed_robot_publisher = self.create_publisher(Int32, '/global/robots/removed', 10)

    def setup_robot(self, robot: Robot):

        """Setup_Robot is called by register_callback upon a successful registration and setup of a new robot.
        This is to be distinguished from the standard setup function 'setup()' in StackControl.ino 

        setup_robot takes parameter robot object as type Robot.
        
        A ROS publisher and subscriber are created specific to this robot:
        
        Robot marker publisher:

            The robot marker publisher topic name is in the format '/robot{robot.id}/markers' where id is an attribute of object robot.
            The publisher facilitates the sending of Int64 values, used to describe the binary representation of ArUco markers.
            The robot's StackControl script on the m5 Stack hosts the corresponding topic subscriber and displays the marker upon receiving a valid Int64.

            create_publisher(Int64, f'/robot{robot.id}/markers', 10)
        
        Robot heartbeat subscriber:

            Creates a unique topic subscriber with topic name in the format '/robot{robot.id}/heartbeat' where id is an attribute of object robot.
            Calls method heartbeat_callback after receiving ping from m5 stack aboard robot.

            create_subscription(Bool, f'/robot{robot.id}/heartbeat', robot.heartbeat_callback, 10)
        
        """
        robot.marker_publisher = self.create_publisher(Int64, f'/robot{robot.id}/markers', 10)
        robot.heartbeat_subscription = self.create_subscription(Bool, f'/robot{robot.id}/heartbeat', robot.heartbeat_callback, 10)
        robot.time_since_heartbeat = 0

        marker = marker_maker.marker_to_int(robot.id)
        time.sleep(3)

        marker_msg = Int64()
        marker_msg.data = marker
        self.logger.log('publishing marker')
        robot.marker_publisher.publish(marker_msg)

    def register_callback(self, msg: Int32):

        """Method register_callback is the callback of the register_subscription subscriber and is called after a robot has successfully completed
        its setup routine.

        register_callback takes a ROS message 'msg' of type Int32 as its parameter.

        Global variable self.next_id tracks next unique ID to assign to robots and agents.

        The contents of 'msg' may be -1 or any other integer, determining two cases.

        Case 1:
            If the data of msg is equal to -1, then the StackControl script on the robot's onboard m5 stack has finished its initial setup
            and is requesting that it is given a unique robot ID.
            In this case, self.next_id bundled into an Int32 message and sent to the robot, after which it is incremented.
        
        Case 2:
            If the data of msg is any other value than -1, then it is interpreted as an acknowledgement of the robot's newly assigned ID.
            This is sent after the second phase of the robot's setup is complete, after having requested its new ID. Msg contains the new ID.
            In this case, the value of msg is compared to all other active IDs and subject to error checking.
            A warning if the ID is a duplicate.

        
        """
        if msg.data == -1:
            self.active_robot_ids.append(self.next_id)
            robot = Robot()
            robot.id = self.next_id
            self.active_robots.append(robot)

            self.logger.log(f'Sending id: {self.next_id}')

            id_msg = Int32()
            id_msg.data = self.next_id
            self.ids_publisher.publish(id_msg)

            self.next_id += 1
        else:
            id_count = self.active_robot_ids.count(msg.data)
            if id_count > 1:
                self.logger.warn(f'Duplicate id: {msg.data}')
            else:
                robot_index = self.active_robot_ids.index(msg.data)
                robot = self.active_robots[robot_index]
                self.setup_robot(robot)

                id_msg = Int32()
                id_msg.data = robot.id
                self.logger.log(f'Publishing new robot')
                self.added_robot_publisher.publish(id_msg)


    def check_alive(self, time_delta):

        """The check_alive method compares the time since each robot's last heartbeat with the time elapsed.
        If the robot has not responded in over a time threshold, then it is defined as no longer alive and method kill_robot is called."""
        for robot in self.active_robots:
            if robot.alive:
                robot.time_since_heartbeat += time_delta

                if robot.time_since_heartbeat > self.death_time * 1000 * 1000 * 1000:
                    self.kill_robot(robot)

    def kill_robot(self, robot: Robot):

        """ Kill_robot takes an object of type Robot as parameter.
        
        It publishes its id to the removed_robot_publisher, then destroys the heartbeat_subscription subscriber and marker_publisher publisher.
        
        Its id is removed from the list of active IDs.
        Finally, the robot itself is removed from the list of active robots."""
        
        id_msg = Int32()
        id_msg.data = robot.id
        self.removed_robot_publisher.publish(id_msg)

        self.destroy_subscription(robot.heartbeat_subscription)
        self.destroy_publisher(robot.marker_publisher)
        self.active_robot_ids.remove(robot.id)
        self.active_robots.remove(robot)


def main():
    rclpy.init()

    server = Server()
    past_time = time.time_ns()

    while rclpy.ok():
        rclpy.spin_once(server, timeout_sec=200)
        current_time = time.time_ns()
        server.check_alive(current_time - past_time)
        past_time = current_time


    rclpy.shutdown()

if __name__ == "__main__":
    main()
