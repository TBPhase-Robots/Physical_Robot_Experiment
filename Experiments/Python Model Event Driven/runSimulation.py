from cv2 import AGAST_FEATURE_DETECTOR_NONMAX_SUPPRESSION
import pygame
import colours
import sys
from pygame.locals import *
import numpy as np
from model.Agent import Agent
import json
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import math
import random
import cv2
from datetime import datetime

import time
import os
#from ProtoInputHandler import ProtoInputHandler
#from model.Listener import Listener
#from model.CommandListener import CommandListener
#from model.AgentListener import AgentListener

from model.ArenaCorner import ArenaCorner

from model.SimulationNode import SimulationNode

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int32

from geometry_msgs.msg import Pose, Vector3

from std_msgs.msg import Float64, Int32, String

from std_msgs.msg import ColorRGBA

from model.PathfindingManager import PathfindingManager


state = "standby_setup_loop"
postedUpdates = 0
sendUpdates = False        

robot_dog_speed = 0
robot_pig_speed = 0
robot_sheep_speed = 0
robot_standby_speed = 0

pack = pygame.sprite.Group()
flock = pygame.sprite.Group()
pigs = pygame.sprite.Group()
standby = pygame.sprite.Group()
agents = pygame.sprite.Group()
arena_corners = pygame.sprite.Group()

config_name = 'defaultConfig2'
rclpy.init(args=None)
# Define the simulation node globally
# The simulation node is a single object with the responsibility of managing all ROS publishers and subscribers. It provides the interface to create new topic when needed.
# Only the single simulation node needs to be polled when looking out for new messages
simulationNode = SimulationNode()

# Configuration file loaded in as global reference:

cfg = 0
with open(f"experiment_config_files/{config_name}.json") as json_file:
    cfg = json.load(json_file)

if ('show_empowerment' not in cfg):
    cfg['show_empowerment'] = True

print(cfg)

# callback invoked by state controller to change the current config file
def SetConfigCallback(data):
    # set the config that we're going to use mid-experiment
    global cfg
    cfgName = data.data

    try:
        with open(f"experiment_config_files/{cfgName}.json") as json_file:
            cfg = json.load(json_file)
            print("changed cfg to ", cfgName)

            state == "standby_setup_loop"
            print("simulation state changed to ", state)

            for agent in agents:
                agent.SetAgentConfig(cfg)

    except:
        print("incorrect file name ", cfgName)


# controller callback is called by an agent whenever it has successfully updated its state via ROS pose
def ControllerCallback(data):
    global postedUpdates
    postedUpdates += 1

# dispatch callback used whenever a command is sent via ROS on the dispatch channel to turn agents from standby to dogs.
def DispatchCallback(data):
    print("DispatchCallback")
    print(data.data)
    # if the user has sent command to dispatch
    if(data.data == "dispatch"):
        # check if there are any agents left in the standby pool
        if(len(standby) > 0):
            # grab the first in agent in the standby pool
            agent = GetAnyAgentFromGroup(standby)
            agent.role = "dog"
            SortAgentsByRole()
    # take a dog off the field and return to standby
    if(data.data == "recall"):
        if(len(pack) > 0):
            agent = GetAnyAgentFromGroup(pack)
            agent.role = "standby"
            SortAgentsByRole()


def GetAnyAgentFromGroup(group):
    for agent in group:
        return agent


# command listener callback is called whenever a new state is recieved by user via the state controller script, utilising CommandListener.py
def CommandListenerCallback(data):
    global state
    print("CommandListenerData:")
    print(data.data)

    # if set all to standby
    if(data.data == "set_to_standby"):
        for agent in agents:
            agent.role = "standby"

        SortAgentsByRole()
    else:
        state = data.data
        print("success")
        print(state)

# Calculates voronoi partition for current flock/pack behaviour
def calc_voronoi_partitioning(flock, pack):
    for dog in pack:
        dog.empty_sub_flock()

    for sheep in flock:
        min_dist = 10000
        for dog in pack:
            dist = np.linalg.norm(sheep.position - dog.position)
            if dist < min_dist:
                min_dist = dist
                sheep.set_closest_dog(dog)
        sheep.closest_dog.add_sheep_to_sub_flock(sheep)
# end function


# Add agent function registers an agent within the simulation environment, with specified ID
def add_agent(agents, position, cfg, id, screen, simulationNode):
    agent = Agent(position=position, id=id, cfg=cfg, rotation=0.0, poseAgentCallback=ControllerCallback,
                  role="agent", screen=screen, simulationNode=simulationNode)
    agents.add(agent)
    add_sound = pygame.mixer.Sound("audio/added_test.mp3")
    pygame.mixer.Sound.play(add_sound)
    agent.PublishForceToTopic(np.array([0.0, 0.0]), screen)
    # agent.role = 'agent'
    return id + 1

# Called whenever the command to add a new agent is recieved. Message contains ID of specified agent. Chooses a random location, then calls add_agent with specified location and ID.
def add_agent_callback(msg):
    global agents

    print("========= callback done")
    print(msg)
    new_id = msg.data

    randx = random.uniform(50, 900)
    randy = random.uniform(50, 550)
    add_agent(agents=agents, position=np.array([randx, randy]), cfg=cfg, id=new_id, screen=screen, simulationNode=simulationNode)

#Draws the statis componments of the world
# This includes background, target area, and soft play area
# Agent start positions are drawn as small dots
def DrawWorld(cfg):
    screen.fill(colours.DGREY)
    pygame.draw.rect(screen, colours.GREY, pygame.Rect(
        0, 0, cfg['world_width'], cfg['world_height']))
    # Draw target box
    pygame.draw.rect(screen, colours.RED, pygame.Rect(
        cfg['target_position'][0] - 100, cfg['target_position'][1] - 100, 200, 200), 3)

    cfg['world_corner_points'] = [np.array([0, 0]),
                                np.array([cfg['world_width'], 0]),
                                np.array([cfg['world_width'], cfg['world_height']]),
                                np.array([9, cfg['world_height']])] 

    points = []
    if cfg['use_arena_corner_markers']:
        arena_corners.update(screen)
        non_zero_count = 0
        for arena_corner in arena_corners:
            points.append(arena_corner.position)
            if (arena_corner.position[0] > 0 or arena_corner.position[1] > 0):
                non_zero_count += 1
        
        if non_zero_count >= 4:
            points.sort(key = lambda x: x[1])
            top_points = [points[0], points[1]]
            top_points.sort(key = lambda x: x[0])
            bottom_points = [points[2], points[3]]
            bottom_points.sort(key = lambda x: x[0], reverse=True)

            points = top_points + bottom_points

            for i in range(1, 5):
                text_surface = my_font.render(f"{i}:{int(points[i-1][0])},{int(points[i-1][1])}", False, (0, 0, 0))
                screen.blit(text_surface, points[i-1])

            cfg['corner_points'] = points
            pygame.draw.line(screen, colours.BLUE, points[0], points[1], 3)
            pygame.draw.line(screen, colours.BLUE, points[1], points[2], 3)
            pygame.draw.line(screen, colours.BLUE, points[2], points[3], 3)
            pygame.draw.line(screen, colours.BLUE, points[3], points[0], 3)
        else:        
            cfg['corner_points'] = [np.array([cfg['play_area_x'], cfg['play_area_y']]),
                                    np.array([cfg['play_area_x'] + cfg['play_area_width'], cfg['play_area_y']]),
                                    np.array([cfg['play_area_x'] + cfg['play_area_width'], cfg['play_area_y'] + cfg['play_area_height']]),
                                    np.array([cfg['play_area_x'], cfg['play_area_y']+cfg['play_area_height']])] 

    else:
        pygame.draw.rect(screen, colours.BLUE, pygame.Rect(
            cfg['play_area_x'], cfg['play_area_y'], cfg['play_area_width'], cfg['play_area_height']), 3)

        cfg['corner_points'] = [np.array([cfg['play_area_x'], cfg['play_area_y']]),
                                np.array([cfg['play_area_x'] + cfg['play_area_width'], cfg['play_area_y']]),
                                np.array([cfg['play_area_x'] + cfg['play_area_width'], cfg['play_area_y'] + cfg['play_area_height']]),
                                np.array([cfg['play_area_x'], cfg['play_area_y']+cfg['play_area_height']])] 
        points = cfg['corner_points']

    offset = 100
    offsets = [np.array([-offset,-offset]),
               np.array([+offset,-offset]),
               np.array([+offset,+offset]), 
               np.array([-offset,+offset])]

    cfg['outer_corner_points'] = [cfg['corner_points'][i] +offsets[i] for i in range(len(points))]        

    pygame.draw.line(screen, colours.RED, cfg['outer_corner_points'][0], cfg['outer_corner_points'][1], 3)
    pygame.draw.line(screen, colours.RED, cfg['outer_corner_points'][1], cfg['outer_corner_points'][2], 3)
    pygame.draw.line(screen, colours.RED, cfg['outer_corner_points'][2], cfg['outer_corner_points'][3], 3)
    pygame.draw.line(screen, colours.RED, cfg['outer_corner_points'][3], cfg['outer_corner_points'][0], 3)

    for i in range(1, 5):
        text_surface = my_font.render(f"{i}:{int(cfg['outer_corner_points'][i-1][0])},{int(cfg['outer_corner_points'][i-1][1])}", False, (0, 0, 0))
        screen.blit(text_surface, cfg['outer_corner_points'][i-1])




    for pos in cfg['initial_sheep_positions']:
        pygame.draw.circle(screen, colours.WHITE, pos, 2)

    for pos in cfg['initial_dog_positions']:
        pygame.draw.circle(screen, colours.BLUE, pos, 2)

    for pos in cfg['standby_positions']:
        pygame.draw.circle(screen, colours.RED, pos, 2)

    # pygame.display.update()
    # pygame.display.flip()


# calls standard behaviour on all sheep and dog agents for simulation
def ExperimentUpdateTimestep(pack, flock, cfg):
    if (len(pack) > 0):
        calc_voronoi_partitioning(flock, pack)
        for dog in pack:
            dog.SimulationUpdate_Dog(screen, flock, pack, agents, cfg)

    else:
        for sheep in flock:
            sheep.closest_dog = None

    if(len(flock) > 0):
        for sheep in flock:

            sheep.SimulationUpdate_Sheep(screen, flock, pack, agents, cfg)

# Function repeatedly called to decide if an agent should move to a destination, or if it already has reached destination.
# Calls MoveToPoint on agent if displacement from target is greater than threshold, otherwise  HaltAgent is called.
# HaltAgent sends a command to physical robot to stop moving
def MoveToPointDecision(agent: Agent, movePos, cfg):
    point_x = movePos[0]
    point_y = movePos[1]
    agentPos = np.array([agent.position[0], agent.position[1]])
    # TODO - Set distance to a variable in the cfg file instead of a magic number
    if(np.linalg.norm(movePos - agentPos) > 85):
        agent.MoveToPoint(point_x=point_x, point_y=point_y,
                          screen=screen, agents=agents, cfg=cfg)
    else:
        agent.HaltAgent(screen=screen)

# Extended version of MoveTpPointDecision. Takes a list of destinations and moves agent towards each in order. Stops and returns 1 when finished path.
# Path is usually defined by points calculated with the pathfinding feature.
def FollowPathDecision(agent: Agent, path, cfg):

    pathFindingTileRadius = cfg['world_width'] / cfg['path_finding_width']

    if(len(path) == 0):
        agent.HaltAgent(screen=screen)
        return 1

    if(len(path) > 0):
        agentPos = np.array([agent.position[0], agent.position[1]])
        targetPos = path[0]

        if(len(path) == 1):
            if(not cfg['event_driven']):
                pathFindingTileRadius = 8

        if(np.linalg.norm(targetPos - agentPos) > pathFindingTileRadius):
            agent.MoveToPoint(
                point_x=targetPos[0], point_y=targetPos[1], screen=screen, agents=agents, cfg=cfg)
        else:
            path = path[1:]
            agent.SetPath(path)

    else:
        agent.HaltAgent(screen=screen)
        return 1

    return 0

# Function moves all agents tagged standby directly to a unique standby position
def StandbySetupUpdateTimestep(agents, cfg):
    # make all agents go to top
    standbyPositions = cfg['standby_positions']
    i = 0
    for agent in agents:
        i += 1
        point = standbyPositions[i]
        # if the distance between target point and self is very low, halt movement and do not send move command
        movePos = np.array(point)
        MoveToPointDecision(agent=agent, movePos=movePos, cfg=cfg)

# Set the role tag of all agents to standby
def SetAllAgentRolesToStandby():
    for agent in agents:
        agent.role = "standby"

# Empty pack, flock, pigs, and standby groups. Resort all agents defined in group Agents into these groups defined by their role tag
def SortAgentsByRole():
    # get maximum robot speeds and transmit
    global robot_dog_speed
    global robot_pig_speed
    global robot_sheep_speed
    global robot_standby_speed

    print("sort agents by role")

    pack.empty()
    flock.empty()
    pigs.empty()
    standby.empty()

    msg = Float64()
    msg.data = 1.0

    colourMsg = ColorRGBA()

    for agent in agents:
        if(agent.role == "dog"):
            pack.add(agent)
            msg.data = robot_dog_speed
            agent.maxSpeedPublisher.publish(msg)
            #print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - DOG")

            # TODO - remove setting agent colour at start
            colourMsg.r = 0.8
            colourMsg.g = 0.8
            colourMsg.b = 1.0
            agent.colourPublisher.publish(colourMsg)

        if(agent.role == "sheep"):
            flock.add(agent)
            msg.data = robot_sheep_speed
            agent.maxSpeedPublisher.publish(msg)
            #print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - SHEEP")
            # TODO - remove setting agent colour at start
            colourMsg.r = 1.0
            colourMsg.g = 1.0
            colourMsg.b = 1.0
            agent.colourPublisher.publish(colourMsg)
            print("setting agent ", str(agent.id),
                  " colour to ", str(colourMsg), " - SHEEP")

        if(agent.role == "pig"):
            pigs.add(agent)
            msg.data = robot_pig_speed
            agent.maxSpeedPublisher.publish(msg)
           # print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - PIG")

            # TODO - remove setting agent colour at start
            colourMsg.r = 1.0
            colourMsg.g = 0.0
            colourMsg.b = 0.0
            agent.colourPublisher.publish(colourMsg)
            print("setting agent ", str(agent.id),
                  " colour to ", str(colourMsg), " - PIG")
        if(agent.role == "standby"):
            standby.add(agent)
            msg.data = robot_standby_speed
            agent.maxSpeedPublisher.publish(msg)
          #  print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - STANDBY")

            # TODO - remove setting agent colour at start
            colourMsg.r = 0.0
            colourMsg.g = 1.0
            colourMsg.b = 0.0
            agent.colourPublisher.publish(colourMsg)
            print("setting agent ", str(agent.id),
                  " colour to ", str(colourMsg), " - STANDBY")


def isInRectangle(centerX, centerY, radius, x, y):

    return x >= centerX - radius and x <= centerX + radius and y >= centerY - radius and y <= centerY + radius

# test if coordinate (x, y) is within a radius from coordinate (center_x, center_y)

# Function used to determine if a user's mouse click is inside one of the agents. Used in the process of user manually removing an agent
def isPointInCircle(centerX, centerY, radius, x, y):
    if(isInRectangle(centerX, centerY, radius, x, y)):
        dx = centerX - x
        dy = centerY - y
        dx *= dx
        dy *= dy
        distanceSquared = dx + dy
        radiusSquared = radius * radius
        return distanceSquared <= radiusSquared
    return False

# Removes an agent by ID. Sends the ID of the deleted agent to the agentRemovalRequest topic
def RemoveAgent(agent, agentRemovalRequestPublisher):
    print("removing agent ", agent.id)
    msg = Int32()
    msg.data = agent.id
    agentRemovalRequestPublisher.publish(msg)
    agents.remove(agent)
    agent.delete()
    SortAgentsByRole()

    add_sound = pygame.mixer.Sound("audio/remove_test.mp3")
    pygame.mixer.Sound.play(add_sound)

def RemoveAgentCallback(message):
    id = message.data
    for agent in agents:
        if agent.id == id:
            RemoveAgent(agent, agentRemovalRequestPublisher)
    
#end functon

def main(show_empowerment=False):

    global screen
    global state
    global postedUpdates
    global sendUpdates

    global robot_dog_speed
    global robot_pig_speed
    global robot_sheep_speed
    global robot_standby_speed

    robot_dog_speed = cfg['robot_dog_max_speed']
    robot_pig_speed = cfg['robot_pig_max_speed']
    robot_sheep_speed = cfg['robot_sheep_max_speed']
    robot_standby_speed = cfg['robot_standby_max_speed']

    end_game = False

    pygame.init()

    pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
    global my_font 
    my_font = pygame.font.SysFont('Comic Sans MS', 30)

    screen = pygame.display.set_mode(
        [cfg['world_width'] + 80, cfg['world_height']])

    # Instantiate the Pathfinding manager
    # The Pathfinding manager is responsible for creating an x*y pathfinding matrix for resetting agent positions between trials
    pathfindingManager = PathfindingManager(screen, cfg)

    agent_id = 0

    # Topic names

    # Change state machine state
    commandListenerTopicName = "/controller/command"
    # Dispatch standby to dog and vice versa
    dispatchListenerTopicName = "/controller/dispatch"
    # Add new agent to simulation
    agentListenerTopicName = "/global/robots/added"
    # Change configuration file by name
    jsonListenerTopicName = "/controller/config"
    # Request queue to remove agent from camera and server after rmoving from simulation
    agentRemovalPublisherTopicName = "/global/agents/removal_requests"
    # Remove agent by ID from simulation
    killListenerTopicName = "/controller/command"

    # define the state command listener:
    commandListener = simulationNode.CreateStringListener(
        commandListenerTopicName, CommandListenerCallback)

    # define the dispatch listener
    dispatchListener = simulationNode.CreateStringListener(
        dispatchListenerTopicName, DispatchCallback)

    # define the add agent listener
    agentListener = simulationNode.CreateAgentListener(
        agentListenerTopicName, add_agent_callback)

    jsonListener = simulationNode.CreateStringListener(
        jsonListenerTopicName, SetConfigCallback)

    global agentRemovalRequestPublisher
    agentRemovalRequestPublisher = simulationNode.CreateIntPublisher(
        agentRemovalPublisherTopicName)

    killListener = simulationNode.create_subscription(Int32, "/global/agents/removed", RemoveAgentCallback, 10)

    if (cfg['use_arena_corner_markers']):
        arena_corners_subscribers = []

        for i in range(100, 104):
            arena_corner = ArenaCorner(i, cfg)
            arena_corners.add(arena_corner)
            sub = simulationNode.create_subscription(Pose, f"arena_corners_{i}", arena_corner.ArenaCornerCallback, 10)
            arena_corners_subscribers.append(sub)
            print(f"create subscriber arena_corners_{i}")

    # put all robots into standby, if any already exist for whatever reason.
    SetAllAgentRolesToStandby()

    SortAgentsByRole()

    pathfindingAgentId = 0

    
    # Main Loop
    while (not end_game):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

            # If the user right clicks on an agent, remove it from simulation
            if(event.type == pygame.MOUSEBUTTONDOWN):
                if(event.button == 3):

                    # get the screen coordinates of the click
                    clickPos = event.pos
                    # if the screen coordinates of the click lie within the bounds of an agent
                    if(len(agents) > 0):
                        closest_agent = None
                        # get boundary of agent
                        for agent in agents:
                            if(isPointInCircle(centerX=agent.position[0], centerY=agent.position[1], radius=cfg['agent_radius'], x=clickPos[0], y=clickPos[1])):
                                # remove agent
                                print(
                                    "REMOVE THE AGENT FROM THE BOTTOM OF THE SCREEN")
                                closest_agent = agent
                                RemoveAgent(
                                    agent, agentRemovalRequestPublisher)

                    # then remove it

        DrawWorld(cfg=cfg)

        # The simulation node is a single object with the responsibility of managing all ROS publishers and subscribers. It provides the interface to create new topic when needed.
        # Only the single simulation node needs to be polled when looking out for new messages
        # -- Poll the simulation node:
        rclpy.spin_once(simulationNode, timeout_sec=0.1)
        
        # If there have been no changes of state in the last execution step as detected by the camera and sent via ROS
        # postedUpdates is incremented by agent position callbacks
        if(postedUpdates > 0):
            sendUpdates = True
            postedUpdates = 0
        else:
            sendUpdates = False

        # if we have recieved an update from one of the agents, then proceed with simulation
        # cfg['event_driven'] is the flag that should be set to true when running the experiment with real robots
        # Only execute the state machine loop if we are running locally or if there is a new detected real world state
        if(sendUpdates or (not cfg['event_driven'])):
            # print("here we are")
            # ===================================== SETUP START==============================#
            if(state == "setup_start"):
                # print("and here we are also in set_up start")

                # we assume all agents are in standby position. Set all agents to standby:
                SetAllAgentRolesToStandby()
                SortAgentsByRole()
                time.sleep(0.2)
                # look at experiment config file
                # choose first n standby agents as sheep
                sheepPositions = cfg['initial_sheep_positions']
                n = len(sheepPositions)
                i = 0

                for agent in standby:
                    agent.role = "sheep"
                    i += 1
                    if(i >= n):
                        break

                # re order the groups of agents
                SortAgentsByRole()
                time.sleep(0.2)

                dogPositions = cfg['initial_dog_positions']
                n = len(dogPositions)
                i = 0
                for agent in standby:
                    agent.role = "dog"
                    i += 1
                    if(i >= n):
                        break

                # re order the groups of agents
                SortAgentsByRole()
                time.sleep(0.2)

                # calculate the amount of reserve dogs left
                reserveDogs = (cfg['max_number_of_dogs'] - len(pack))

                # subtract the amount of reserve dogs from the standby agents. The remaining n should go to the pigsty
                pigAmount = len(standby) - reserveDogs

                if(pigAmount > 0):
                    # assign n pigs to the pigsty from the standby pool
                    i = 0
                    for agent in standby:
                        agent.role = "pig"
                        i += 1
                        if(i >= pigAmount):
                            break

                # re order the groups of agents
                SortAgentsByRole()
                time.sleep(0.5)

                stationaryAgents = [pack, standby, pigs]
                pathfindingManager.GenerateWorldMatrix(
                    stationaryAgents, sheepPositions)

                i = 0
                for sheep in flock:
                    pos = sheepPositions[i]
                    #sheep = GetAnyAgentFromGroup(flock)
                    sheep.SetPath(pathfindingManager.FindPath(sheep, pos))
                    i += 1

                # advance the state machine loop to the next state
                state = "sheep_setup_loop"
                pathfindingAgentId = 0


            # ===================================== SHEEP SETUP LOOP ===============================#
            if(state == "sheep_setup_loop"):
                #print("here we are in sheep setup loop")

                # get the list of positions for sheep to move to

                if(cfg['sequential_pathfinding']):
                    pathfindingAgentId = pathfindingManager.SequentialPathfindingStep(
                        pathfindingAgentId, flock, agents, cfg, FollowPathDecision, cfg['initial_sheep_positions'])

                else:
                    # simultaneous
                    for sheep in flock:
                        FollowPathDecision(
                            agent=sheep, path=sheep.path, cfg=cfg)

            # ===================================== DOG SETUP START ===============================#
            if(state == "dog_setup_start"):

                i = 0
                stationaryAgents = [flock, standby, pigs]
                dogPositions = cfg['initial_dog_positions']
                pathfindingManager.GenerateWorldMatrix(
                    stationaryAgents, dogPositions)

                for dog in pack:
                    pos = dogPositions[i]
                    dog.SetPath(pathfindingManager.FindPath(dog, pos))
                    i += 1

                state = "dog_setup_loop"
                pathfindingAgentId = 0

            # ===================================== DOG SETUP LOOP ===============================#
            if(state == "dog_setup_loop"):

                if(cfg['sequential_pathfinding']):
                    pathfindingAgentId = pathfindingManager.SequentialPathfindingStep(
                        pathfindingAgentId, pack, agents, cfg, FollowPathDecision, cfg['initial_dog_positions'])
                else:
                    for dog in pack:
                        FollowPathDecision(agent=dog, path=dog.path, cfg=cfg)

            # ===================================== PIG SETUP START ===============================#
            if(state == "pig_setup_start"):
                i = 0
                stationaryAgents = [flock, standby, pack]
                pigPositions = cfg['pigsty_positions']
                pathfindingManager.GenerateWorldMatrix(
                    stationaryAgents, pigPositions)

                for pig in pigs:
                    pos = pigPositions[i]
                    pig.SetPath(pathfindingManager.FindPath(pig, pos))
                    i += 1

                state = "pig_setup_loop"
                pathfindingAgentId = 0
            # ===================================== PIG SETUP LOOP ===============================#
            if(state == "pig_setup_loop"):
                #print("here we are in pig setup")
                if(cfg['sequential_pathfinding']):
                    pathfindingAgentId = pathfindingManager.SequentialPathfindingStep(
                        pathfindingAgentId, pigs, agents, cfg, FollowPathDecision, cfg['pigsty_positions'])
                else:
                    for pig in pigs:
                        FollowPathDecision(agent=pig, path=pig.path, cfg=cfg)
            # ===================================== STANDBY SETUP START ===============================#
            if(state == "standby_setup_start"):
                #print("here we are in standby start")
                i = 0
                standbyPositions = cfg['standby_positions']
                stationaryAgents = [flock, pigs, pack]
                pathfindingManager.GenerateWorldMatrix(
                    stationaryAgents, standbyPositions)
                for agent in standby:
                    pos = standbyPositions[i]
                    agent.SetPath(pathfindingManager.FindPath(agent, pos))
                    i += 1

                state = "standby_setup_loop"
                pathfindingAgentId = 0
            # ===================================== STANDBY SETUP LOOP ===============================#
            if(state == "standby_setup_loop"):
                #print("here we are in standby loop")
                for agent in standby:
                    FollowPathDecision(agent=agent, path=agent.path, cfg=cfg)
            
            # ===================================== EXPERIMENT ===============================#
            elif(state == "experiment"):
                #print("here we are in experiment")
                ExperimentUpdateTimestep(pack=pack, flock=flock,  cfg=cfg)
                StandbySetupUpdateTimestep(agents=standby, cfg=cfg)
                for pig in pigs:
                    pig.HaltAgent(screen)
            for agent in agents:
                agent.DrawSelf(screen)
        # if we are not cleared to send updates due to no incoming data (if in event driven mode), then draw all agents
        else:

            for agent in agents:
                agent.DrawSelf(screen)

        # Redraw the pygame display
        pygame.display.flip()


# end function
if __name__ == '__main__':
    main()
