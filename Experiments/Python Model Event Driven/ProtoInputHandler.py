from itertools import count
import pygame
import numpy as np
import colours
import sys
from model.Agent import Agent
import random
import math
from model.Dog import Dog
from model.Sheep import Sheep

class ProtoInputHandler:


    
    
    def __init__(self, DogAgentStart, SheepAgentStart):
        self.packTransforms = []
        self.flockTransforms = []



    def LoadAgentTransforms(self, pack, flock):
        dog_positions = []
        for dog in pack: 
            dog_tuple = []
            dog_tuple.append(dog.id)
            dog_tuple.append(dog.position)
            # rotation placeholder
            dog_tuple.append(0)
            dog_positions.append(dog_tuple)
        sheep_positions = []
        for sheep in flock: 
            sheep_tuple = []
            sheep_tuple.append(sheep.id)
            sheep_tuple.append(sheep.position)
            # rotation placeholder
            sheep_tuple.append(0)
            sheep_positions.append(sheep_tuple)


     #   print("LoadAgentTransforms")

        self.packTransforms = dog_positions
        self.flockTransforms = sheep_positions
       # print(self.packTransforms)
      #  print(self.flockTransforms)

    def RandomiseAgentTransforms(self, maxRange):
       

        for i in range (len(self.packTransforms)):
            # transform defined by ID, position(x,y), rotation
            transform = self.packTransforms[i]
            id = transform[0]
            position = transform[1]
            x = position[0]
            y = position[1]
            rotation = transform[2]
            
            new_x = x + random.uniform(-maxRange, maxRange)
            new_y = y + random.uniform(-maxRange, maxRange)
            newPosition = []
            newPosition.append(new_x)
            newPosition.append(new_y)

            newRotation = rotation + random.uniform(-maxRange, maxRange)

            newTransform = [id, newPosition, newRotation]
            self.packTransforms[i] = newTransform
     
            
            

    def GetAgentTransforms(self):
    #    print("SetAgentTransforms")
        return self.packTransforms

    def Call(self, data):

        print("called ProtoInput handler with + " , data)