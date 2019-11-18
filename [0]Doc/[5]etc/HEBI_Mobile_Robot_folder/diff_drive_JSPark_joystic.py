#!/usr/bin/env python3

#This is a very basic example to drive a diff-drive robot using a phone with the Hebi app.

import hebi
from time import sleep
import numpy as np
from numpy.distutils.fcompiler import none
import time
from math import pi
import pygame
import xbox360_controller

#wait for network
sleep(10)

# Create lookup
lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate, and print out its contents
sleep(2)

for entry in lookup.entrylist:
    print(entry)

# Create a group
group = lookup.get_group_from_names(['HebiRobot'], ['_LeftWheel', '_RightWheel'])

while group == None:
    print('Wheels not found.')
    group = lookup.get_group_from_names(['HebiRobot'], ['_LeftWheel', '_RightWheel'])
print('Found group on network: size {0}'.format(group.size))

group.command_lifetime = 100
group_command = hebi.GroupCommand(group.size)

Left = 0
Right = 0
TurnScale = 0
TurnLimit = 0.5
r = 0.1
L = 0.4
simulation_flag = 0

Vmax = 0.85

pygame.init()
# make a controller (should this be in the game loop?)
controller = xbox360_controller.Controller(0)

while True:
    pygame.event.get()

    # joystick stuff
    pressed = controller.get_buttons()

    a_btn = pressed[xbox360_controller.A]
    b_btn = pressed[xbox360_controller.B]
    y_btn = pressed[xbox360_controller.Y]

    lt_x, lt_y = controller.get_left_stick()
    rt_x, rt_y = controller.get_right_stick()

    w = rt_x
    v = -lt_y
    abort = a_btn
    simulation = b_btn   
    if y_btn:
        simulation_flag = 0

    v = v * Vmax 
    w = w * (2*Vmax/L)


    if v >= 0: 
        Left  = 1/r * v -L/2/r * (-w)
        Right = 1/r * v +L/2/r * (-w) 
    else:    
        Left  = 1/r * v -(-L/2/r * (-w))
        Right = 1/r * v -(+L/2/r * (-w))

    if abort == 1:
        group_command.velocity = [0, 0]
        group.send_command(group_command)
        exit(0)

    if simulation == 1:
        if simulation_flag == 0:
            start = time.time()
            end = time.time()
            simulation_flag = 1
        else:
            end = time.time()

        duration = end - start
        print(duration)

        if duration <= 3:
            v = 0.5
            w = 0.0
            Left  = 1/r * v -L/2/r * (-w)
            Right = 1/r * v +L/2/r * (-w) 
        elif duration > 3 and duration <= 4:
            v = 0.0
            w = -pi/2.0
            Left  = 1/r * v -L/2/r * (-w)
            Right = 1/r * v +L/2/r * (-w) 
        elif duration > 4 and duration <= 5:
            v = 0.5
            w = 0.0
            Left  = 1/r * v -L/2/r * (-w)
            Right = 1/r * v +L/2/r * (-w) 
        else:
            #group_command.velocity = [0, 0]
            #group.send_command(group_command)
            #exit(0)
            Left  = 0
            Right = 0


    group_command.velocity = [Left, -Right]
    group.send_command(group_command)


