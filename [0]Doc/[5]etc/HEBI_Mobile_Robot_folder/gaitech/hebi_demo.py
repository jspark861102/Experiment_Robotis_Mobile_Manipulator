#!/usr/bin/env python3

import sys
import hebi
import numpy as np
from time import sleep
import pygame

pygame.init()

screen = pygame.display.set_mode((500, 300))
pygame.display.set_caption("Hebi Keyboard Control")
clock = pygame.time.Clock()

# Create lookup
lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate, and print out its contents
sleep(2)

for entry in lookup.entrylist:
    print(entry)

group = lookup.get_group_from_names(['HebiRobot'], ['_LeftWheel', '_RightWheel'])

if not group:
    print('Group not found: Did you forget to set the module family and names above?')
    exit(1)

print('Found group on network: size {0}'.format(group.size))

# Sets the command lifetime to 100 milliseconds
# group.command_lifetime = 100

group_command = hebi.GroupCommand(group.size)

mode = 'stop'
Left = 0.0
Right = 0.0
SpeedScale = 1.0


while True:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                mode = 'go'
            if event.key == pygame.K_x:
                mode = 'back'
            if event.key == pygame.K_a:
                mode = 'left'
            if event.key == pygame.K_d:
                mode = 'right'
                Left = SpeedScale/2
                Right = -SpeedScale/2
            if event.key == pygame.K_s:
                mode = 'stop'
                SpeedScale = 1.0
            if event.key == pygame.K_q:
                SpeedScale = SpeedScale + 0.5
            if SpeedScale > 5.0:
                SpeedScale = 5.0
            if event.key == pygame.K_e:
                SpeedScale = SpeedScale - 0.5
            if SpeedScale < 1.0:
                SpeedScale = 1.0

    if mode == 'go':
        Left = SpeedScale
        Right = SpeedScale
    if mode == 'back':
        Left = -SpeedScale
        Right = -SpeedScale
    if mode == 'left':
        Left = -SpeedScale / 2
        Right = SpeedScale / 2
    if mode == 'right':
        Left = SpeedScale / 2
        Right = -SpeedScale / 2
    if mode == 'stop':
        Left = 0.0
        Right = 0.0

    group_command.velocity = [-Right, Left]
    group.send_command(group_command)