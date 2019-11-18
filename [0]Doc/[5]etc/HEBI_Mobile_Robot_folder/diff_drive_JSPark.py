#!/usr/bin/env python3

#This is a very basic example to drive a diff-drive robot using a phone with the Hebi app.

import hebi
from time import sleep
import numpy as np
from numpy.distutils.fcompiler import none
import time

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

phoneGroup = lookup.get_group_from_names(['HEBI'], ['Virtual IO'])
    
while phoneGroup == None:
    print('Phone not found - set phone name to \"Virtual IO\" and family to \"HEBI\"')
    phoneGroup = lookup.get_group_from_names(['HEBI'], ['Virtual IO'])

print('Found group on network: size {0}'.format(group.size))
print('Found group on network: size {0}'.format(phoneGroup.size))

group_fbk = hebi.GroupFeedback(phoneGroup.size)

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
from math import pi

while True:
    group_fbk = phoneGroup.get_next_feedback(reuse_fbk=group_fbk)
    
    if group_fbk == None:
        print("Phone connetion lost!")
        x = 0
        y = 0
        abort = 0
    else:    
        io_a = group_fbk.io.a
        io_b = group_fbk.io.b
        w = io_a.get_float(7)[0]
        v = io_a.get_float(2)[0]
        abort = io_b.get_int(1)[0]
        simulation = io_b.get_int(5)[0]
#    	vel_lim1 = io_b.get_int(2)[0]
#    	vel_lim2 = io_b.get_int(3)[0]

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
            group_command.velocity = [0, 0]
            group.send_command(group_command)
            exit(0)


    group_command.velocity = [Left, -Right]
    group.send_command(group_command)