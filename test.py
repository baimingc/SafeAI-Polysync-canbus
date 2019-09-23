import pygame
import random
import numpy as np
import math
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_BACKQUOTE
from pygame.locals import K_BACKSPACE
from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_TAB
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_c
from pygame.locals import K_d
from pygame.locals import K_h
from pygame.locals import K_m
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_w
from pygame.locals import K_MINUS
from pygame.locals import K_EQUALS

pygame.init()

done = False
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

width = 200
height = 200

screen = pygame.display.set_mode([width, height])
pygame.display.set_caption("Autonomous agents")
clock = pygame.time.Clock()

class Steer:
    def __init__(self, x=100, y=100, color=BLACK, radius=80):
        self.color = color
        self.radius = radius
        self.center = [x, y]

    def show(self, angle = 0):
        pygame.draw.circle(screen, self.color, [100, 100], self.radius, 5)
        pygame.draw.circle(screen, self.color, self.center, 20)
        angle = np.pi*angle/180
        x1 = -np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) + self.center[0]
        y1 = -np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) * np.tan(angle) + self.center[1]
        x2 = np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) + self.center[0]
        y2 = np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) * np.tan(angle) + self.center[1]
        R = np.array([[0,1],[-1,0]])
        coord_origin = np.array([x1-self.center[0], y1-self.center[1]])
        coord = np.matmul(R, coord_origin) + np.array(self.center)
        coord = list(coord)
        pygame.draw.line(screen, self.color,[x1,y1], [x2,y2], 10)
        pygame.draw.line(screen, self.color, self.center, coord, 10)


steer = Steer()
angle = -540
while not done:
    clock.tick(30)
    screen.fill(WHITE)
    angle = angle+1
    steer.show(angle=angle)

    pygame.display.flip()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        mainScreenPressed = pygame.key.get_pressed()
        if mainScreenPressed[K_q]:
            exit()

