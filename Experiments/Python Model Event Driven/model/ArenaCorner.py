from ast import Delete
import numpy as np
import pygame
import colours

class ArenaCorner(pygame.sprite.Sprite):

    def __init__(self, id, cfg) -> None:
        pygame.sprite.Sprite.__init__(self)

        self.id = id
        self.position = np.zeros(2)
        self.cfg = cfg
        self.past_positions = [np.zeros(2) for i in range(20)]
    #end function

    def update(self, screen):
        self.past_positions.pop(0)
        self.past_positions.append(self.position)
        xcount = 0
        ycount = 0
        for i in range(0, 20):
            xcount += self.past_positions[i][0]
            ycount += self.past_positions[i][1]
        x = xcount / 20
        y = ycount / 20

        pygame.draw.circle(screen, colours.PINK, [x, y], 5)
    #end function

    def ArenaCornerCallback(self, message):
        widthRatio = self.cfg['world_width'] / self.cfg['cam_width']
        heightRatio = self.cfg['world_height'] / self.cfg['cam_height']
        x = message.position.x * widthRatio
        y = message.position.y * heightRatio
        self.position = np.array([x, y])
    #end function