from logging import root
import pygame
import numpy as np
import colours
import sys
from model.Agent import Agent
import random
import math
from math import degrees, atan2
from model.Sheep import Sheep

import numpy.linalg as LA


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

from model.Listener import Listener

from geometry_msgs.msg import Pose


# Enable an agent 
def EnableAgentCallback(self, msg):
    # decode position and rotation data, set agent position and rotation
    self.callback(msg)

# call this function from central poller
def RosUpdate(self):
    rclpy.spin_once(self.listener, timeout_sec=0)