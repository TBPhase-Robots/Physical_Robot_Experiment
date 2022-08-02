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

from model.Listener import Listener
from model.VectorPublisher import VectorPublisher
from model.FloatPublisher import FloatPublisher
from model.ColourPublisher import ColourPublisher
from model.SimulationNode import SimulationNode
from geometry_msgs.msg import Pose, Vector3



class Agent(pygame.sprite.Sprite):
    
    
    def __init__(self, position, id, cfg, rotation, poseAgentCallback, role, screen, simulationNode) -> None:
        
        pygame.sprite.Sprite.__init__(self)
        
        # generic
        self.direction = np.array([1, 0])
        self.rotation = rotation
        self.callback = poseAgentCallback
        self.position = position
        self.id = id
        self.cfg = cfg
        self.role = role
        self.halted = False
        self.cameraWidth = cfg['cam_width']
        self.cameraHeight = cfg['cam_height']
        self.worldWidth = cfg['world_width']
        self.worldHeight = cfg['world_height']
        self.screen = screen
        self.lastRotation = rotation
        self.hasNewRotation= False
        self.simulationNode = simulationNode

        # width ratio = worldWidth / camera width
        self.widthRatio = self.worldWidth / self.cameraWidth

        # height ratio = worldheight / camera height
        self.heightRatio = self.worldHeight / self.cameraHeight

        # dog
        self.sub_flock = pygame.sprite.Group()
        self.choice_tick_count = 0
        self.target_sheep = None
        self.driving_point = np.zeros(2)
        self.state = 'collecting'
        self.steering_point = np.zeros(2)
        self.empowerment = 0
        

        # sheep 
        self.closest_dog = None
        self.grazing = True
        self.grazing_direction = np.array([1, 0])

        # ROS
        self.topicPoseName = f'/robot{id}/pose'
        self.listener = self.simulationNode.CreatePoseListener(self.topicPoseName, self.AgentCallback) 

        self.topicName = f'/robot{id}/vectors'
        print("self.topic_name: ", self.topicName)
        self.vectorPublisher = self.simulationNode.CreateVectorPublisher(self.topicName)

        self.colourTopicName = f'/robot{id}/colours'
        self.colourPublisher = self.simulationNode.CreateColourPublisher(self.colourTopicName)
        

        # NOTE 
        # THIS IS NOT YET IMPLEMENTED ON THE ROBOT END
        self.floatTopicName = f'/robot{id}/speed'
        self.maxSpeedPublisher = self.simulationNode.CreateFloatPublisher(self.floatTopicName)
        
        

    #end function
    
    def SetAgentConfig(self, newCfg):
        self.cfg = newCfg

        self.cameraWidth = newCfg['cam_width']
        self.cameraHeight = newCfg['cam_height']
        self.worldWidth = newCfg['world_width']
        self.worldHeight = newCfg['world_height']

        # width ratio = worldWidth / camera width
        self.widthRatio = self.worldWidth / self.cameraWidth

        # height ratio = worldheight / camera height
        self.heightRatio = self.worldHeight / self.cameraHeight

        print("successfully changed config of agent ", self.id)

    def AgentCallback(self, msg):
        # decode position and rotation data, set agent position and rotation
        newPose = msg

        pose_x = newPose.position.x
        pose_y = newPose.position.y

        # we need to convert real world pose into screen space

        # camera width
        camWidth = self.cameraWidth

        # camera height
        camHeight = self.cameraHeight

        # screen width
        screenWidth = self.worldWidth

        # screen height
        screenHeight = self.worldHeight

        # width ratio = worldWidth / camera width
        widthRatio = screenWidth / camWidth

        # height ratio = worldheight / camera height
        heightRatio = screenHeight / camHeight

        x = pose_x * widthRatio
        y = pose_y * heightRatio

        # draw line in forward vector
        # as debug

        # calculate forward vector
       
        

      #  print('-------')
      #  print(x, y, "    ", str(self.id))
     #   print(pose_x, pose_y, "    ", str(self.id))
     #   print(newPose.orientation.z, "    ", self.id)


        self.position[0] = x
        self.position[1] = y
        self.rotation = newPose.orientation.z

        forwardX = math.cos(self.rotation)
        forwardY = math.sin(self.rotation)

      #  print(np.add(self.position, np.array([forwardX, forwardY])*80), "    ", self.id)

        # x = pose_x * widthRatio
        # y = pose_y * heightRatio

        linex = self.position[0] + forwardX *50
        liney = self.position[1] + forwardY * 50

        lineEnd = np.array([linex, liney])

        pygame.draw.line(self.screen, colours.YELLOW, [x, y], [linex, liney] ,5)

        #pygame.display.flip()
        

        self.callback(msg)

        self.hasNewRotation = True




    





    # a and b must be np arrays
    def CalcAngleBetweenVectors(self, a, b):
        # NORMALISE A AND B
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)
        dot = np.dot(a, b)
        if (dot > 1):
            dot = 1
        theta = np.arccos(dot)
        if ((np.cross([a[0], a[1], 0], [b[0], b[1], 0])[2] > 0)   ):
            theta = - theta
        return math.degrees(theta)

   


    def CalcBearing(x, y, center_x, center_y):
        angle = degrees(atan2(y - center_y, x - center_x))
        bearing1 = (angle + 360) % 360
        bearing2 = (90 - angle) % 360
        return bearing1, bearing2
    #end function 


    def DrawSelf(self, screen):
        # debug for now
        # calculate forward vector
        forwardX = math.sin(self.rotation)
        forwardY = math.cos(self.rotation)

        radius = self.cfg['agent_radius']
       # pygame.draw.line(self.screen, colours.RED, self.position, np.add(self.position, np.array([forwardX, -forwardY])*80) ,5)

        if(self.role == "dog"):
            pygame.draw.circle(screen, colours.BLUE, self.position, radius)
        elif(self.role == "sheep"):
            pygame.draw.circle(screen, colours.WHITE, self.position, radius)
        elif(self.role == "standby"):
            pygame.draw.circle(screen, colours.GREEN, self.position, radius)
        elif(self.role == "pig"):
            pygame.draw.circle(screen, colours.PINK, self.position, radius)
        else:
            pygame.draw.circle(screen, colours.BLACK, self.position, radius)

       


    # publishes a force to the agent's corresponding robot 
    # the force is scaled to the screen space - camera space ratio
    def PublishForceToTopic(self, force, screen):
        vForce = Vector3()
        # switched at some point
        vForce.x = force[0] 
        vForce.y = force[1] 
        vForce.z = self.rotation
        
        # since we have now updated the rotation of the robot, set to false
        self.hasNewRotation = False
      #  print("publishing Vforce ", vForce, " to ", self.topicName)

        self.vectorPublisher.publish(vForce)
        pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, np.array([vForce.x, vForce.y])*40 ), 2)
      #  print("sending rotation ", vForce.z, " to robot ", self.id)

      

    # halts an agent's movement in the real world 
    # communicated via sending a [0,0] vector
    # a flag remains signalling the robot has been halted until a new command is sent
    def HaltAgent(self, screen):
        if(not self.halted):
        #    print("agent ", self.id, " has been halted")
            self.PublishForceToTopic(np.array([0.0,0.0]), screen)
            self.halted = True

    # called by the function MoveToPointDecision() in runSimulation.py
    # linearly moves agent (with obstacle avoidance) to point
    # MoveToPointDecision() stops calling this when in a target range
    # Publishes resultant force to robot topic
    def MoveToPoint(self, point_x, point_y, screen, agents, cfg):
        # this is a valid move command
        # setting this flag to false ensures that we are not halted
        # halted agents are not sent commands in runSimulation.py
        self.halted = False
        moveRepelDistance = cfg['move_to_point_repel_distance']



        # point is np array [0] [1] as x,y
        point = np.array([point_x, point_y])
        self.steering_point = point

        # calculate force to steeringpoint
        force = self.steering_point -self.position 
        force = force / np.linalg.norm(force)

        brakingDistance = cfg['robot_move_to_point_braking_distance']

        curDistance = np.linalg.norm(self.steering_point -self.position)
        if(curDistance > brakingDistance):
            forceMultiplier = curDistance/brakingDistance
            force*= forceMultiplier

        # calculate repulsion force from all other agents
        F_A = np.zeros(2)

        objectAvoidance = False
        i = 0
        for agent in agents:
            if (agent.id != self.id):
                if (np.linalg.norm((self.position) - (agent.position)) < moveRepelDistance):

                    # get counter clockwise orthogonal vector between self and agent
                   # distanceVector = np.array([self.position[0] - agent.position[1], self.position[1] - agent.position[1]])

                    # rotation matrix
                   # theta = np.radians(90)
                   # c, s = np.cos(theta), np.sin(theta)
                   # R = np.array(((c, -s), (s, c)))
                   # print(R)
                   # orthogonalVector = distanceVector @ R
                   # print("orthogonal vector: ", orthogonalVector)

                    # get repulsion vector (negative force + force * ((moveRepelDistance - (np.linalg.norm((self.position) - (agent.position)))  / moveRepelDistance)
                   # repulsionVector = -force + force* ((moveRepelDistance - (np.linalg.norm(self.position - agent.position))) /moveRepelDistance)
                    # sum vectors

                   # orthogonalVector = orthogonalVector / np.linalg.norm(orthogonalVector)

                   # F_A += (orthogonalVector * 5) + repulsionVector


                    objectAvoidance = True
                   # i += 1



                    F_A = cfg['agent_repulsion_from_agents'] * np.add(F_A, (self.position - agent.position) / np.linalg.norm(self.position - agent.position))
        if(objectAvoidance):
            F_A = (F_A / np.linalg.norm(F_A)) * i
            repulsionForce = F_A #+ (0.75 * np.array([F_A[1], -F_A[0]]))
            moveForce = force + repulsionForce

            if(np.linalg.norm(moveForce) < 0.1):
                moveForce = moveForce + force*0.1

        else:
            moveForce = force 

        # if the simulation is running using real world robots, don't move the agent
        if(not cfg['event_driven_lock_movements']):
            self.position = np.add(self.position, moveForce)
        
        
        
        # publish force to robot topic
        self.PublishForceToTopic(moveForce, screen) 

        if(not cfg['event_driven_lock_movements']):
            self.position = np.add(self.position, moveForce)

        # do collision detection if not running with real robots
        collision_check = True
        if(not cfg['event_driven_lock_movements']):
            while (collision_check):
                collision_check = False
                for agent in agents:
                    if (agent.id != self.id):
                        if (np.linalg.norm(self.position - agent.position) <= cfg['agent_radius'] * 2):
                            self.position = np.add(self.position, (self.position - agent.position)/3)
                            collision_check = True

        if(cfg['realistic_agent_movement_markers']):
            pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, np.array(moveForce)) ,4)




    # Function describes all normal dog behaviour for the agent
    # Called by runSimulation.py when in experiment state
    def SimulationUpdate_Dog(self, screen, flock, pack, agents ,cfg):
        self.halted = False
        target = cfg['target_position']
        if (len(self.sub_flock) > 0):
            sheep_positions = []
            for sheep in flock:
                sheep_positions.append(sheep.position)
            C = Agent.calcCoM(self, sheep_positions)
            furthest_sheep_position = C
            if (self.choice_tick_count == 0):
                self.driving_point = np.add(C, cfg['driving_distance_from_flock_radius'] * (C - target) / np.linalg.norm(C - target))
                for sheep in self.sub_flock:
                    if (np.linalg.norm(sheep.position - C) > np.linalg.norm(furthest_sheep_position - C)):
                        furthest_sheep_position = sheep.position
                        self.target_sheep = sheep
            try:
                furthest_sheep_position = self.target_sheep.position
            except:
                furthest_sheep_position = C

            # determine whether the dog should be driving or collecting:
            # driving behaviour pushes flock towards goal
            # collecting behaviour orbits flock and rounds up stragglers
            if (self.choice_tick_count == 0):
                if (np.linalg.norm(furthest_sheep_position - C) < cfg['collection_radius']):
                    self.state = 'driving'
                else:
                    self.state = 'collecting'
            # steering point is the closest thing to moving towards a target point to which the agent should drive
            if (self.state == 'driving'):
                self.steering_point = self.driving_point
            elif (self.state == 'collecting'):
                self.steering_point = np.add(furthest_sheep_position, cfg['collection_distance_from_target_sheep'] * (furthest_sheep_position - C) / np.linalg.norm(furthest_sheep_position - C))
        else:
            self.state = 'unassigned'
            sheep_positions = []
            for sheep in flock:
                sheep_positions.append(sheep.position)
            C = Agent.calcCoM(self, sheep_positions)
            furthest_sheep_position = C
            
            if (self.choice_tick_count == 0):
                for sheep in flock:
                    if (np.linalg.norm(sheep.position - C) > np.linalg.norm(furthest_sheep_position - C)):
                        furthest_sheep_position = sheep.position
            
            outer_flock_radius_point = np.add(C, np.linalg.norm(C - furthest_sheep_position) * ((C - target) / np.linalg.norm(C - target)))
            self.steering_point = np.add(outer_flock_radius_point, cfg['driving_distance_from_flock_radius'] * ((C - target) / np.linalg.norm(C - target)))

        # calculate the force to drive towards the flock
        F_H = self.calc_F_H_Dog(screen, cfg, self.steering_point, flock)
        # calculate the repulsion force from other dogs
        F_D = self.calc_F_D_Dog(pack)
        # apply force coefficients from the configuration file
        F = (cfg['dog_forces_with_flock'] * F_H) + (cfg['dog_repulsion_from_dogs'] * F_D)

        if (cfg['debug_dog_forces']):
            pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_dogs'] * F_D), 8)
        if (cfg['debug_steering_points']):
            pygame.draw.circle(screen, colours.BLACK, self.steering_point, 4)

        # publish force to topic
        self.PublishForceToTopic(F, screen)


        ## on screen stuff ========================================================================

        # calculate forward vector
        forwardX = math.sin(self.rotation)
        forwardY = math.cos(self.rotation)

        if(cfg['realistic_agent_movement_markers']):
            # black line is target rotation
            pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, np.array(F)*10) ,8)
            # draw line in forward vector
            pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*80) ,5)

        # calculate angle between current dir and target dir:
        angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), np.array(F))



        # Omnidrive movement
        if(not cfg['realistic_agent_movement']):
            if(not cfg['event_driven_lock_movements']):
                self.position = np.add(self.position, F)
        # Differntial drive movement
        # Do not move if doing a real world simulation
        elif(not cfg['event_driven_lock_movements']):
            # differential drive rotation towards target direction

            # rotate until forward vector is parallel to force within reason

            # if vector is parallel, then go forward

            # for now, the magic number 5 is used to check the angle bounds. This is in degrees
            if(angle > 5):
                self.rotation -= 0.1
                self.position = np.add(self.position, [2*forwardX, -2*forwardY])
            elif(angle < -5):
                self.rotation += 0.1
                self.position = np.add(self.position, [2*forwardX, -2*forwardY])
            else:
                self.position = np.add(self.position, F)

            # at this point, we would attempt to transmit this agent's movement command to the bot via ROS
            
            # we should transmit the current position, current rotation, target position. 
            # The robot will attempt to drive to the position, then keep going

        self.choice_tick_count += 1
        if (self.choice_tick_count >= cfg['ticks_per_choice']):
            self.choice_tick_count = 0

        collision_check = True

        # if we are simulating locally (ie event_driven_lock_movements is set to off), do some rudimentary collision checks and update dog positions if so
        if(not cfg['event_driven_lock_movements']):
            while (collision_check):
                collision_check = False
                for agent in agents:
                    if (agent.id != self.id):
                        if (np.linalg.norm(self.position - agent.position) <= cfg['agent_radius']):
                            self.position = np.add(self.position, (self.position - agent.position)/2)
                            collision_check = True
                
        

        # Calculate empowerment values, taking into account sheep of a range of up to 50 units from dog agent
        if (cfg['empowerment_type'] == 0):
            self.empowerment = len(self.sub_flock)
        elif (cfg['empowerment_type'] == 1):
            if (len(self.sub_flock) > 0):
                self.empowerment = 5
            else:
                self.empowerment = 0
            for sheep in flock:
                if (np.linalg.norm(self.position - sheep.position) <= 50):
                    self.empowerment += 5 - math.floor(np.linalg.norm(self.position - sheep.position) / 10)

        if (cfg['debug_dog_states']):
            if (self.state == 'driving'):
                pygame.draw.circle(screen, colours.DRIVE, self.position, 5)
            elif (self.state == 'collecting'):
                pygame.draw.circle(screen, colours.COLLECT, self.position, 5)
            else:
                pygame.draw.circle(screen, colours.BLUE, self.position, 5)
        else:
            if (cfg['show_empowerment']):
                if (self.empowerment < 5):
                    colour = np.array([155 + round(100 * self.empowerment / 5), 0, 0])
                elif (self.empowerment < 10):
                    colour = np.array([255, round(255 * (self.empowerment - 5) / 5), 0])
                elif (self.empowerment < 15):
                    colour = np.array([255 - round(255 * (self.empowerment - 10) / 5), 255, 0])
                elif (self.empowerment < 20):
                    colour = np.array([0, 255 - round(100 * (self.empowerment - 15) / 5), 0])
                else:
                    colour = np.array([0, 155, 0])
                pygame.draw.circle(screen, colour, self.position, 5)
            else:
                pygame.draw.circle(screen, colours.BLUE, self.position, 5)

        if (cfg['debug_sub_flocks']):
            if (self.id < 5):
                pygame.draw.circle(screen, colours.SRANGE[self.id], self.position, 4)
            else:
                pygame.draw.circle(screen, colours.BLACK, self.position, 4)
    #end function

    def empty_sub_flock(self):
        self.sub_flock.empty()
    #end function

    def add_sheep_to_sub_flock(self, sheep):
        self.sub_flock.add(sheep)
    #end function


    # calculates dog-dog repulsion force vector
    def calc_F_D_Dog(self, pack):
        F_D_D = np.zeros(2)
        for dog in pack:
            if (dog.id != self.id):
                if (np.array_equal(self.position, dog.position)):
                    F_D_D = np.add(F_D_D, (self.position - dog.position) / np.linalg.norm(self.position - dog.position))

        F_D = F_D_D + (0.75 * np.array([F_D_D[1], -F_D_D[0]]))
        return F_D
    #end function

    def sine_step(self, theta):
        if ((-math.pi < theta and -math.pi / 2 >= theta) or (math.pi < theta and 3 * math.pi / 2 >= theta)):
            return 1
        elif ((-3 * math.pi / 2 < theta and -math.pi >= theta) or (math.pi / 2 < theta and math.pi >= theta)):
            return -1
        else:
            return -math.sin(theta)
    #end function

    # Calculate Sheep-Dog force interaction

    def calc_F_H_Dog(self, screen, cfg, steering_point, flock):
        sheep_positions = []
        if (len(self.sub_flock) > 0):
            for sheep in self.sub_flock:
                sheep_positions.append(sheep.position)
        else:
            for sheep in flock:
                sheep_positions.append(sheep.position)
        C = Agent.calcCoM(self, sheep_positions)
        W = steering_point

        R_C_D = (self.position - C) / np.linalg.norm(self.position - C)
        R_C_W = (W - C) / np.linalg.norm(W - C)

        dot = np.dot(R_C_D, R_C_W)
        if (dot > 1):
            dot = 1
        theta_D_C_W = np.arccos(dot)
        if (np.cross([R_C_D[0], R_C_D[1], 0], [R_C_W[0], R_C_W[1], 0])[2] < 0):
            theta_D_C_W = - theta_D_C_W

        R_D_W = (W - self.position) / np.linalg.norm(W - self.position)
        R_D_T = np.array([R_C_D[1], -R_C_D[0]]) 

        H_F = 1 - math.exp(-2 * abs(math.degrees(theta_D_C_W)))
        H_T = self.sine_step(theta_D_C_W)

        sum = np.zeros(2)
        for sheep in flock:
            sum = np.add(sum, (self.position - sheep.position) / (2 *np.linalg.norm(self.position - sheep.position)))

        F_F = H_F * sum
        F_W = R_D_W
        F_T = H_T * R_D_T

        if (cfg['debug_dog_forces']):
            pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_sheep'] * F_F), 8)
            pygame.draw.line(screen, colours.RED, self.position, np.add(self.position, 10 * cfg['dog_attraction_to_steering_point'] * F_W), 8)
            pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, 10 * cfg['dog_orbital_around_flock'] * F_T), 8)
        F_H = (cfg['dog_repulsion_from_sheep'] * F_F) + (cfg['dog_attraction_to_steering_point'] * F_W) + (cfg['dog_orbital_around_flock'] * F_T)

        return F_H
    #end function


    # calculate the centre of mass of given group of agent vectors
    def calcCoM(self, vector_list):
        #Calculates the centre of mass as the average position of the
        #vectors listed in vector_list
        #vector_list = [x1,y1;x2,y2;....]

        if np.any(vector_list):
            V = np.atleast_2d(vector_list)
            N = V.shape[0]
            com = np.sum(vector_list,axis=0)/N
        else:
            com = np.array([])
        return com
    #end function


    # ================================ SHEEP =================================#



    # Function describes all normal sheep behaviour for the agent
    # Called by runSimulation.py when in experiment state
    def SimulationUpdate_Sheep(self, screen, flock, pack, agents, cfg):
        self.halted = False
        # calculate forward vector
        forwardX = math.sin(self.rotation)
        forwardY = math.cos(self.rotation)



        # if there is a dog within our vision range, do not exhibit grazing behaviour, RUN AWAY!
        if (self.closest_dog != None):
            if (np.linalg.norm(self.position - self.closest_dog.position) <= cfg['sheep_vision_range']):
                self.grazing = False
            else:
                if (random.random() < 0.05):
                    self.grazing_direction = np.array([random.uniform(-3, 3), random.uniform(-3, 3)])
                self.grazing = True
                if(cfg['event_driven']):
                    self.HaltAgent(screen=screen)
        # if we are safe from dog agents, then graze
        else:
            self.grazing = True
            if (random.random() < 0.05):
                self.grazing_direction = np.array([random.uniform(-3, 3), random.uniform(-3, 3)])

        # TODO update grazing behaviour over network

        # This section defines all of the grazing behaviour of an agent when it is a sheep
        if (self.grazing):
            self.halted = False

            if (len(flock) > 1):
                F_S = self.calc_F_S_Sheep(flock, cfg)
            else:
                F_S = 0

            # TODO - grazing behavuour over network
            if(cfg['event_driven']):
                    self.HaltAgent(screen=screen)


            # TODO
            # update repulsion to other sheep to be based on forces
            if(not cfg['event_driven_lock_movements']):
                self.position = np.add(self.position, (cfg['sheep_repulsion_from_sheep'] * F_S))



            # calculate the angle between current sheep bearing and target sheep grazing direction
            angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), self.grazing_direction)
            if(cfg['realistic_agent_movement_markers']):
                # black line is target rotation
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, self.grazing_direction*5) ,8)
                # draw line in forward vector
                pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*30) ,5)

            # differntial grazing drive (if local simulation)
            if (random.random() < cfg['grazing_movement_chance'] and not cfg['event_driven_lock_movements'] ):
                if(not cfg['realistic_agent_movement']):
                    self.position = np.add(self.position, self.grazing_direction)
                else:
                    if(angle > 10):
                        self.rotation -= 0.4
                        self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                    elif(angle < -10):
                        self.rotation += 0.4
                        self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                    else:
                        self.position = np.add(self.position, self.grazing_direction)
        
        # if the sheep is not grazing, exhibit the following behaviour:
        else:
            # we are no longer halted, there is adog in range
            self.halted = False
            # calculate repulsion force of all dogs on the sheep
            F_D = self.calc_F_D_Sheep(pack, cfg)
            if (len(flock) > 1):
                # calculate repulsion force from other sheep agents
                F_S = self.calc_F_S_Sheep(flock, cfg)
                # calculate attraction force to other sheep agents
                F_G = self.cal_F_G_Sheep(flock, cfg)
            else:
                F_S = 0
                F_G = 0

            F = (cfg['sheep_resulsion_from_dogs'] * F_D) + (cfg['sheep_repulsion_from_sheep'] * F_S) + (cfg['sheep_attraction_to_sheep'] * F_G)


            # check for bounds outside of the play area
            # the play area is a rectangle defined in the config file - it is a soft area in which sheep agents should try to remain inside.
            x = self.position[0]
            y = self.position[1]
            playAreaLeftBound = cfg['play_area_x']
            playAreaTopBound = cfg['play_area_y']

            playAreaRightBound = playAreaLeftBound + cfg['play_area_width']
            playAreaBottomBound = playAreaTopBound + cfg['play_area_height']
            outOfBounds = False
            boundaryForce = np.array([0.0,0.0])
            # if outside of the play area, add a force
            r = random.uniform(-1, 1)
            if(x < playAreaLeftBound):
                outOfBounds = True
                boundaryForce += np.array([2.0,r])
                print("agent too left at position ", x, y)
            if(x > playAreaRightBound):
                outOfBounds = True
                boundaryForce += np.array([-2.0, r])
                print("agent too right at position ", x, y)
            if(y < playAreaTopBound):
                outOfBounds = True
                print("agent too high at position ", x, y)
                boundaryForce += np.array([r, 2.0])
            if( y > playAreaBottomBound):
                outOfBounds = True
                print("agent too low at position ", x, y)
                boundaryForce += np.array([r, -2.0])

            # if outside of the play area, add an overwhelming force to return back inside it
            F = np.add(boundaryForce*40, F)

            # publish F to topic
            print("sheepForce: ",  F)
            print("sheepForce magnitude", np.linalg.norm(F))

            self.PublishForceToTopic(F, screen)

            angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), np.array(F))
            if(cfg['realistic_agent_movement_markers']):
                # black line is target rotation
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, self.grazing_direction*8) ,8)
                # draw line in forward vector
                pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*40) ,5)

            # local omni drive movement
            if(not cfg['realistic_agent_movement'] and not cfg['event_driven_lock_movements']):
                self.position = np.add(self.position, F)

            # local differential drive movement
            elif(not cfg['event_driven_lock_movements']):
                if(angle > 10):
                    self.rotation -= 0.2
                    self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                elif(angle < -10):
                    self.rotation += 0.2
                    self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                else:
                    if(np.linalg.norm(F) > 10 and not outOfBounds):
                        F /= (np.linalg.norm(F) /10)
                    
                    self.position = np.add(self.position, np.array(F))

            if (cfg['debug_sheep_forces']):
                pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['sheep_resulsion_from_dogs'] * F_D), 8)
                pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, 10 * cfg['sheep_repulsion_from_sheep'] * F_S), 8)
                pygame.draw.line(screen, colours.RED, self.position, np.add(self.position, 10 * cfg['sheep_attraction_to_sheep'] * F_G), 8)

        collision_check = True

        # TODO
        # update to differential drive and force behaviour
        if(not cfg['event_driven_lock_movements']):
            while (collision_check):
                collision_check = False
                for agent in agents:
                    if (agent.id != self.id):
                        if (np.linalg.norm(self.position - agent.position) <= cfg['agent_radius'] *2):
                            self.position = np.add(self.position, (self.position - agent.position)/2)
                            collision_check = True


        

        #super().update(screen)
        if (cfg['debug_sheep_states']):
            if (self.grazing):
                pygame.draw.circle(screen, colours.GRAZE, self.position, 5)
                
            else:
                pygame.draw.circle(screen, colours.HERD, self.position, 5)
        if (cfg['debug_sub_flocks']):
            if (self.closest_dog != None):
                if (self.closest_dog.id < 5):
                    pygame.draw.circle(screen, colours.SRANGE[self.closest_dog.id], self.position, 4)
                else:
                    pygame.draw.circle(screen, colours.BLACK, self.position, 4)
    #end function

    def set_closest_dog(self, dog):
        self.closest_dog = dog
    #end function

    def calc_F_D_Sheep(self, pack, cfg):
        sum = np.zeros(2)
        for dog in pack:
            direction = self.position - dog.position
            magnitude = np.linalg.norm(direction)
            if (magnitude != 0):
                sum += (direction / magnitude) * math.exp(- cfg['lambda_D'] * magnitude)
        return sum
    #end function

    def calc_F_S_Sheep(self, flock, cfg):
        sum = np.zeros(2)
        for sheep in flock:
            if (sheep.id != self.id):  
                direction = self.position - sheep.position
                magnitude = np.linalg.norm(direction)
                if (magnitude != 0):
                    sum += (direction / magnitude) * math.exp(- cfg['lambda_S'] * magnitude)
        return sum
    #end function

    def cal_F_G_Sheep(self, flock, cfg):
        sheep_positions_ordered = []
        for sheep in flock:
            if (sheep.id != self.id):
                if not sheep_positions_ordered:
                    sheep_positions_ordered.append(sheep.position) #if list is empty
                else:
                    for i in range(0, len(sheep_positions_ordered)):
                        if (np.linalg.norm(sheep.position - self.position) < np.linalg.norm(self.position - sheep_positions_ordered[i])):
                            sheep_positions_ordered.insert(i, sheep.position)
                            break
                        else:
                            if (i == len(sheep_positions_ordered) - 1):
                                sheep_positions_ordered.append(sheep.position)

        social_group_positions = sheep_positions_ordered[:cfg['no_of_sheep_in_social_group']]      
        external_group_positions = sheep_positions_ordered[cfg['no_of_sheep_in_social_group']:]                            

        sheep_positons = []
        for sheep in flock:
            if (sheep.id != self.id):
                sheep_positons.append(sheep.position)

        C = Agent.calcCoM(self, sheep_positons)
        C_i = Agent.calcCoM(self, social_group_positions)
        C_i_prime = Agent.calcCoM(self, external_group_positions)
        
        C_direction = C - self.position
        C_magnitude = np.linalg.norm(C_direction)

        if (cfg['lambda_G'] > 0):
            C_i_direction = C_i - self.position
            C_i_magnitude = np.linalg.norm(C_i_direction)
            F_G = (cfg['lambda_G'] * (C_i_direction / C_i_magnitude)) + ((1 - cfg['lambda_G']) * (C_direction / C_magnitude))
        else:
            if (len(external_group_positions) > 0):
                C_i_prime_direction = C_i_prime - self.position
                C_i_prime_magnitude = np.linalg.norm(C_i_prime_direction)
                F_G = (-cfg['lambda_G'] * (C_i_prime_direction / C_i_prime_magnitude)) + ((1 + cfg['lambda_G']) * (C_direction / C_magnitude))
            else:
                F_G = 0

        return F_G
    #end function





