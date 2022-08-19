from cmath import pi
from turtle import forward
import pygame
import colours
import numpy as np
from logging import root
import pygame
import numpy as np
import colours
import sys

import random
import math
from math import degrees, atan2


import numpy.linalg as LA


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

#from model.Listener import Listener
#from model.VectorPublisher import VectorPublisher
#from model.FloatPublisher import FloatPublisher
#from model.ColourPublisher import ColourPublisher
from model.SimulationNode import SimulationNode
from geometry_msgs.msg import Pose, Vector3


from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder


class PathfindingManager():

    # Pathfinding manager is the object responsible for calculating agent paths during the setup phases between each trial.

    def __init__(self, screen, cfg):
        print("pathfinding engage")

        self.cfg = cfg
        self.pathFindingWidth = cfg['path_finding_width']
        self.pathFindingHeight = cfg['path_finding_height']
        self.worldWidth = cfg['world_width']
        self.worldHeight = cfg['world_height']
        self.pathFindingWorld_x = int(self.pathFindingWidth)
        self.pathFindingWorld_y = int(self.pathFindingHeight)
        self.pathFindingGridSquareWidth = self.worldWidth / self.pathFindingWidth
        self.pathFindingGridSquareHeight = self.worldHeight / self.pathFindingHeight
        self.screen = screen 
        self.worldMatrix = [0,1]

    # Takes a pair of screenspace coordinates and a pair of tile coordinates (pathfinding space). Returns true if the screen coordinates are inside the tile, otherwise returns false
    def IsInTile(self, screenCoordinates, tileCoordinates):

        tileTopLeft_x = int(self.pathFindingGridSquareWidth * tileCoordinates[1])
        tileTopLeft_y = int(self.pathFindingGridSquareHeight * tileCoordinates[0])

        if(screenCoordinates[0] > tileTopLeft_x and screenCoordinates[0] < tileTopLeft_x+self.pathFindingGridSquareWidth  and screenCoordinates[1] > tileTopLeft_y and screenCoordinates[1] < tileTopLeft_y + self.pathFindingGridSquareHeight):
            return True
        else:
            return False

    # Takes a pair of screen coordinates and returns a pair of pathfinding coordinates
    # It's worth remembering that the returned value is in format [x,y], but the pathfinding matrix addresses elements [y,x]
    def ConvertScreenPosToPathFindingPos(self, pos):
        print("ConvertScreenPosToPathFindingPos")
        pathFindingPosX = int(pos[0] / self.pathFindingGridSquareWidth)
        pathFindingPosY = int(pos[1] / self.pathFindingGridSquareHeight)

        # world matrix is addressed y,x
        # but return x y in grid
        return [pathFindingPosX, pathFindingPosY]


    # Takes a list of target positions for a group of agents, an agent role, and all other registered agents
    # Any agents without the specified role that are in the way are added to a list and returned
    def GetAgentsInTheWay(self, targetPositions, agentRole, agents):
        robotsInTheWay = []
        for pos in targetPositions:
            sheepPositionTile = self.ConvertScreenPosToPathFindingPos(pos)
            for agent in agents:
                if(not agent.role == agentRole):
                    agentPositionTile = self.ConvertScreenPosToPathFindingPos(agent.position)
                    if(agentPositionTile[0] == sheepPositionTile[0] and agentPositionTile[1] == sheepPositionTile[1] ):
                        robotsInTheWay.append(agent)  

        return robotsInTheWay    

    # Sequential pathfinding step for setting up agents between trials
    # Sequential pathfinding avoids problems of multi agent collision detection but is slower overall
    # Takes the pathfinding id of a subject agent (the agent currently moving), a list of other agents that are waiting to move/have moved, a list of all agents in the simulation, config file, a callback to the FollowPathDecision function in runSimulation.py, and the current position of all agents
    def SequentialPathfindingStep(self, pathfindingAgentId, movingAgents, allAgents, cfg, FollowPathDecision, agentPositions):
        agents = []
        currentAgentId = 0
        for agent in movingAgents:
            agents.append(agent)
            if(pathfindingAgentId < len(agents)):
                stationaryAgents = [allAgents]
                self.GenerateWorldMatrix(stationaryAgents, agentPositions)
                currentSheep = agents[pathfindingAgentId]
                currentAgentId = currentSheep.id
                finished = FollowPathDecision(agent= currentSheep, path=currentSheep.path, cfg=cfg)
                if(finished == 1):
                    print("sheep reached destination")
                    print(pathfindingAgentId)
                    pathfindingAgentId = pathfindingAgentId + 1

        for agent in agents:
            if(not agent.id == currentAgentId):
                agent.HaltAgentOverride(self.screen)
        return pathfindingAgentId


    # Generates a x*y matrix describing the current simualtion space. The matrix is usually of considerably lower resolution than pixel space
    # x and y of the matrix are defined by:
    # self.pathFindingWidth = cfg['path_finding_width']
    # self.pathFindingHeight = cfg['path_finding_height']
    # ^ defined in config file and set in __init()__ method upon object instantiation

    # Returns the resulting matrix and also sets it as the locally stored matrix variable attached to self.
    # Empty tiles in the matrix are defined as +1, obstacles are defined as -1

    # The world matrix is addressed in [y,x] format, as opposed to (x,y)

    # Takes a concatenated list of agent groups, as well as a list of endpoints
    def GenerateWorldMatrix(self, stationaryAgentsTuple, endPoints):

        stationaryAgents = []
        for group in stationaryAgentsTuple:
            for agent in group:
                stationaryAgents.append(agent)

        np.set_printoptions(threshold=np.inf)

        # generate numpy matrix
        worldMatrix = np.ones((self.pathFindingWorld_y ,self.pathFindingWorld_x ), dtype=float)
        agentRadius = self.cfg['agent_radius']

        

        for x in range(self.pathFindingWorld_x):
            for y in range (self.pathFindingWorld_y):
                curX = x * self.pathFindingGridSquareWidth
                curY = y * self.pathFindingGridSquareHeight

                agentsInTile = 0
                endpoint = 0



                # check if there is an agent in the current square
                for agent in stationaryAgents:
                    if(agent.position[0] > curX and agent.position[0] < curX+self.pathFindingGridSquareWidth  and agent.position[1] > curY and agent.position[1] < curY + self.pathFindingGridSquareHeight):
                        agentsInTile = 1

                        if(agentsInTile > 0):
                            worldMatrix[y, x] = -1

                            # get screen pos, add agent radius up, down, left, and right
                            screenDown =  np.array([agent.position[0], agent.position[1] + agentRadius*1.4])
                            screenUp = np.array([agent.position[0], agent.position[1] - agentRadius*1.4])
                            
                            screenLeft = np.array([agent.position[0] - agentRadius*1.4, agent.position[1]])
                            screenRight = np.array([agent.position[0] + agentRadius*1.4, agent.position[1]])


                            screenUpRight = np.array([agent.position[0] + agentRadius, agent.position[1] - agentRadius])

                            # check if world up, down left and right are in tiles
                            if(y < self.pathFindingWorld_y -1):
                                if(self.IsInTile(screenCoordinates = screenDown, tileCoordinates=[y+1, x])):
                                    worldMatrix[y+1, x] = -1

                            if(y > 0):
                                if(self.IsInTile(screenCoordinates = screenUp, tileCoordinates=[y-1, x])):
                                    worldMatrix[y-1, x] = -1

                            if(x < self.pathFindingWorld_x -1):
                                if(self.IsInTile(screenCoordinates = screenRight, tileCoordinates=[y, x+1])):
                                    worldMatrix[y, x+1] = -1

                            if(x > 0):
                                if(self.IsInTile(screenCoordinates = screenLeft, tileCoordinates=[y, x-1])):
                                    worldMatrix[y, x-1] = -1

                for point in endPoints:
                    if(point[0] > curX and point[0] < curX+self.pathFindingGridSquareWidth  and point[1] > curY and point[1] < curY + self.pathFindingGridSquareHeight):
                        endpoint = 1


                if(endpoint > 0):
                    worldMatrix[y, x] = -1
                # if so, add 1 to the numpy matrix


                    



                    # add 1 to adjacent tiles
                    #if(x < 15):
                    #    worldMatrix[y, x+1] = 1
                    #if(x > 0):
                    #    worldMatrix[y, x-1] = 1
                    #if(y < 8):
                    #    worldMatrix[y+1, x] = 1
                    #if(y > 0):
                    #    worldMatrix[y-1, x] = 1
        #print(worldMatrix)

        self.worldMatrix = worldMatrix

        return worldMatrix

    #def FindNearestEmptyTile(self, agent, )


    # Unimplemented function
    def UpdatePositionOnWorldMatrix(self, agent):
        print("UpdatePositionOnWorldMatrix")


    # Findpath takes an agent and a screen space target position [x,y] and returns a list of points in world space describing the path to its target with respect to the last computed worldMatrix   
    def FindPath(self, agent, targetPos):
        # convert agent position in screen space to pathfinding space

        agentPos = agent.position
        pathFindingPosX = int(agentPos[0] / self.pathFindingGridSquareWidth)
        pathFindingPosY = int(agentPos[1] / self.pathFindingGridSquareHeight)



        #print("pathfinding coordinates ", pathFindingPosX, " ", pathFindingPosY)

        # convert target position to pathfinding space
        pathFindingTargetPosX = int(targetPos[0] / self.pathFindingGridSquareWidth)
        pathFindingTargetPosY = int(targetPos[1] / self.pathFindingGridSquareHeight)

        if(pathFindingTargetPosX >= self.pathFindingWidth):
            pathFindingTargetPosX = self.pathFindingWidth - 1
        if(pathFindingTargetPosX < 0):
            pathFindingTargetPosX = 0

        if(pathFindingTargetPosY >= self.pathFindingHeight):
            pathFindingTargetPosY = self.pathFindingHeight - 1
        if(pathFindingTargetPosY < 0):
            pathFindingTargetPosY = 0

        #print("pathfinding target coordinates ", pathFindingTargetPosX, " ", pathFindingTargetPosY)
        
        lastTileValue = self.worldMatrix[pathFindingPosY, pathFindingPosX]
        # if agent shares tile with stationary:
        self.worldMatrix[pathFindingPosY, pathFindingPosX] = 1
        self.worldMatrix[pathFindingTargetPosY, pathFindingTargetPosX] = 1
        # calculate path
       

        grid = Grid(matrix=self.worldMatrix)

        startPos = grid.node(pathFindingPosX, pathFindingPosY)

        endPos = grid.node(pathFindingTargetPosX, pathFindingTargetPosY)

        finder = AStarFinder(diagonal_movement=DiagonalMovement.only_when_no_obstacle)
        
        print(startPos)
        print(endPos)
        print(grid)
        path, runs = finder.find_path(startPos, endPos, grid)
        print("here")
        print('operations:', runs, 'path length:', len(path))
        print(grid.grid_str(path=path, start=startPos, end=endPos))

        print("path", path)

        

        # set current pathfinding position back to what it once was

        self.worldMatrix[pathFindingPosY, pathFindingPosX] = lastTileValue
        self.worldMatrix[pathFindingTargetPosY, pathFindingTargetPosX] = -1

        # return midpoints of screen space of each tile in the path
        screenCoordinatesPath = []

        for pathfindingCoordinates in path:
            x = pathfindingCoordinates[0] * self.pathFindingGridSquareWidth + self.pathFindingGridSquareWidth * 0.5
            y = pathfindingCoordinates[1] * self.pathFindingGridSquareHeight+ self.pathFindingGridSquareHeight * 0.5
            screenCoordinatesPath.append([x,y])


        screenCoordinatesPath.append([targetPos[0], targetPos[1]])
        print("screen path ", screenCoordinatesPath)

        

        return screenCoordinatesPath



