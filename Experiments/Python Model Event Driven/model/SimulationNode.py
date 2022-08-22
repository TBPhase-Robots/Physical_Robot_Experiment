"""
Code description: Code is used to manage ROS publishers and subscribers for the robot control simulator.
As a result of using a single node, the simulation node, only one ROS node needs to be polled to check for incoming messages.

An instance of SimulationNode is instantiated on startup by runSimulation.
"""


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

# Simulation node extends the ROS node class. It is the single object that manages all publishers and subscribers for the networked robot and local simulation.

class SimulationNode(Node):

    def __init__(self):

        
        rId = random.randint(0, 999999)
        pubName = f'SimulationNode{rId}'
        
        super().__init__(pubName)
        print("created node ", pubName)
    
    
    def CreateAgentListener(self, topic_name, controller_callback):
        """ Creates the subscriber responsible for adding new agents to the simulation.
            Messages are of type std_msgs.msg.Int32; containing the ID of a new agent to add.
            
            
            Parameters:
            topic_name: Name of the topic to listen to. In normal operation, the name of the topic is '/global/robots/added'. 
            
            controller_callback: The method to call once a message has been received. In normal operation, the method is add_agent_callback in the runSimulation class. 


                        add_agent_callback

            """
        print("created subscription: " , topic_name)
        sub = self.create_subscription(Int32, topic_name, controller_callback, 10)
        return sub

    def CreateStringListener(self, topic_name, controller_callback):
        """ Creates a generic subscriber responsible for listening to strings on a specified topic
            Messages are of type std_msgs.msg.String.
            
            
            Parameters:
            topic_name: Name of the topic to listen to. In normal operation, the topics that are created by this method are:


                        "/controller/dispatch" - used to dispatch and recall standby and dog agents.

                        "/controller/config" - used to change the JSON config file by name during operation.
            
            controller_callback: The method to call once a message has been received. In normal operation, the methods that are called by the above subscribers are:


                        runSimulation.DispatchCallback

                        runSimulation.SetConfigCallback
            """

        print("created subscription: " , topic_name)
        sub = self.create_subscription(String, topic_name, controller_callback, 10)
        return sub


    #   topic_name = f'/robot{id}/pose'
    def CreatePoseListener(self, topic_name, controller_callback):
        

        """ Creates the subscriber responsible for handling incoming pose data for a specific agent.

            Messages are of type geometry_msgs.msg.Pose; containing (x,y) position and orientation data.

            A Pose Listener (subscriber) is created each time an agent is added to the simulation, unique to the agent.
            
            This is called in the initialisation method in the Agent.
            
            
            Parameters:
            topic_name: Name of the topic to listen to. In normal operation, the name of the topic is '/robot{id}/pose' (without the {} braces).
            
            controller_callback: The method to call once a message has been received. In normal operation, the method is Agent.AgentCallback in the Agent class. 


                        Agent.AgentCallback

            In turn, this callback calls the method runSimulation.ControllerCallback post an update flag to the main simulation control loop, thereby facilitating event driven behaviour.


                        runSimulation.ControllerCallback

            """



        print("created subscription: " , topic_name)
        sub = self.create_subscription(Pose, topic_name, controller_callback, 10)
        return sub


    # =========PUBLISHERS============#

    def CreateFloatPublisher(self, topic_name):



        print("created publisher: " , topic_name)
        pub = self.create_publisher(Float64, topic_name, 10)
        return pub

    def CreateIntPublisher(self, topic_name):

        """ Creates a publisher responsible for pushing integer messages to a specified ROS topic.

            Messages are of type std_msgs.msg.Int32.
            
            
            Parameters:
            topic_name: Name of the topic to listen to. In normal operation, the topic created by this method is:


                        "/global/agents/removal_requests" - used to send removal requests to the Robot Server once an agent has been successfully removed from the simulation.


        """


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
      