import queue
from turtle import position
from bs4 import Tag
# import rospy2 as rospy

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import String

from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import String, Int32

from typing import List

import numpy as np

class Position():
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

class Robot():
    subscription: Subscription = None
    publisher: Publisher = None
    position: Position = Position()
    id: int = -1

    def position_callback(self,pos):
        x = pos.position.x
        y = pos.position.y
        theta = pos.orientation.z

        self.position.x = x
        self.position.y = y
        self.position.theta = theta


class forward_or_back(Node):
    robots: List[Robot] = []

    def __init__(self):

        # self.robot_subs = {}
        # self.robot_pos = {}
        # self.robot_pubs = {}
        # self.IDs = []

        # rospy.init_node("hello_world")
        super().__init__("hello_world")

        # rospy.on_shutdown(self.shutdown)

        # self.robot_ID = rospy.Subscriber("/global/robots/added",Int32,callback=self.get_IDs)
        self.robot_ID_subscription = self.create_subscription(Int32, "/global/robots/added", self.get_IDs, 10)
        #self.robot_pub = rospy.Publisher("/vectors",Vector3,queue_size=10)

        # while not rospy.is_shutdown():
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if len(self.robots) == 0:
            # elif len(self.robot_pos) == 0: # waits until at least a singular bit of data has been recieved has been idenfied
                 print("No robots")
            else:
                self.robot_control()

 
    def get_IDs(self,msg):

        print("Recieved ID: ", msg.data)
        
        new_robot = Robot()
        new_robot.id = msg.data

        pose_topic = f"/robot{new_robot.id}/pose"
        new_robot.subscription = self.create_subscription(Pose, pose_topic, new_robot.position_callback, 10)

        vectors_topic = f"/robot{new_robot.id}/vectors"
        new_robot.publisher = self.create_publisher(Vector3, vectors_topic, 10)

        self.robots.append(new_robot)
        

    # def new_subscriber(self,ID):
        # tag = "/robot" + str(ID) + "/pose"
        # self.robot_subs[ID] = rospy.Subscriber(tag,Pose,callback=self.position_callback,callback_args=ID)

    # def new_publisher(self,ID):
        # tag = "/robot" + str(ID) + "/vectors"
        # self.robot_pubs[ID] = rospy.Publisher(tag, Vector3, queue_size=10)

    def robot_control(self):
    # For the moment, the code will just check where the robot is and then say back or forward depending on what half it is in
        for robot in self.robots: # get each robots ID number

            message = Vector3()
            # x_pos = robot.position.x
            # y_pos = robot.position.y # gets the robots x and y position
            # theta_pos = robot.position.theta

            # this is where to add that robots command
            # eg_command = [-x_pos,-y_pos]
            # vector = eg_command

            # coordinates must be transformed into the robots frame: use rotation matrix

            # theta = theta_pos
            # theta = 0.0

            # rot_mat = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])

            # new_vec = np.matmul(rot_mat,vector) # this should be 



            message.x = -robot.position.x
            # message.x = 1.0
            message.y = -robot.position.y
            # message.y = 1.0
            message.z = robot.position.theta
            # message.x = new_vec[0]
            # message.y = new_vec[1]

            # print(new_vec)

            # try:
            # if(not message.x == 0.0):
            # print(message.x, message.y, message.z)
            robot.publisher.publish(message)
            # except KeyError:
            #     self.new_publisher(robot)
            #     self.robot_pubs[robot].publish(message)

    # def shutdown(self):
    #     message = Vector3()
    #     message.x, message.y = 0,0
    #     self.robot_pub.publish(message)

if __name__ == "__main__":
    rclpy.init()

    go = forward_or_back()
