#!/usr/bin/env python3
import hebi
from time import sleep
import numpy as np
from numpy.distutils.fcompiler import none
import time
from math import pi
import math as m
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
group.feedback_frequency = 100
Left = 0
Right = 0

# Mobile platform parameters
r = 0.1 #wheel radius
L = 0.4 #width btw axle
D = 0.3 #wheel base (distance btw actuators and caster)

# Simulation parameters
simulation_flag = 0
dt = 0.01
Vmax_manual = 0.85
Vmax_simulation = 0.5
road_width = 0.7
ref_num = 0

circle_r = (road_width - L * m.cos(pi/4)) / (1-m.cos(pi/4))
print(circle_r)

# time for 1/4 circle travel
tf = round(pi *circle_r/(2*Vmax_simulation) +0.5,2)
t = np.linspace(0,tf,tf/dt+1)

# simulation reference
Tnumber = np.linspace(0,1,21)
TRefa = -np.cos(Tnumber * pi)/2+0.5
TRefb = np.cos(Tnumber * pi)/2+0.5
SSRef = np.ones(len(t)-2*len(Tnumber))
Ref = np.concatenate( (np.concatenate((TRefa,SSRef)),TRefb) )
v_ref = 1*Vmax_simulation * Ref
w_ref = 1*(Vmax_simulation/circle_r) * Ref

s_ref = np.zeros(len(v_ref))
t_ref = np.zeros(len(w_ref))

def get_fbk(group):
  fbk = group.get_next_feedback()
  if fbk is None:
    print("Couldn't get feedback")
    raise hell
  return fbk

pygame.init()
controller = xbox360_controller.Controller(0)
fbk = get_fbk(group)
iPos = fbk.position 
while True:
    pygame.event.get()
    fbk = get_fbk(group)

    # joystick setting
    pressed = controller.get_buttons()
    a_btn = pressed[xbox360_controller.A]
    b_btn = pressed[xbox360_controller.B]
    y_btn = pressed[xbox360_controller.Y]
    lt_x, lt_y = controller.get_left_stick()
    rt_x, rt_y = controller.get_right_stick()
    
    # joystic mapping
    w = -rt_x
    v = -lt_y
    abort = a_btn
    simulation = b_btn   
    if y_btn:
        simulation_flag = 0

    # input scaling with max value
    v = v * Vmax_manual
    w = w * (2*Vmax_manual/L)figure;plot(b(:,3))

    Left  = 1/r * v -L/2/r * w
    Right = 1/r * v +L/2/r * w

    group_command.velocity = [Left,-Right]              

    if abort == 1:
        group_command.velocity = [0, 0]
        group.send_command(group_command)
        exit(0)  

    # simulation with reference
    if simulation == 1:
        if simulation_flag == 0:
            start = time.time()
            end = time.time()
            simulation_flag = 1
        else:
            end = time.figure;plot(b(:,3))
        duration = end figure;plot(b(:,3))
        print(duration)figure;plot(b(:,3))

        if ref_num >= len(v_ref)-1: #reference time end
            group_command.effort = [0.5,0.3]                         
        else: #do for reference time
            s_ref[ref_num+1] = s_ref[ref_num] + v_ref[ref_num]*dt
            t_ref[ref_num+1] = t_ref[ref_num] + w_ref[ref_num]*dt

            position = fbk.position - iPos
            s_fbk = r/2*(position[0] + (-position[1]))            
            t_fbk = r/L*(-position[0] + (-position[1]))
            
            v_fbk = r/2*(fbk.velocity[0] + (-fbk.velocity[1]))            
            w_fbk = r/L*(-fbk.velocity[0] + (-fbk.velocity[1]))            
            
            FV = -30*(v_fbk - v_ref[ref_num])
            MV = -30*(w_fbk - w_ref[ref_num])                        
            F = -1800*(s_fbk - s_ref[ref_num]) -1*(v_fbk - v_ref[ref_num])
            M = -400*(t_fbk - t_ref[ref_num]) -1*(w_fbk - w_ref[ref_num])                                                
            #v = -10*(s_fbk - s_ref[ref_num])
            #w = -10*(t_fbk - t_ref[ref_num])                        
            #v = v_ref[ref_num]
            #w = w_ref[ref_num]                      
            
            #Left  = 1/r * v -L/2/r * w
            #Right = 1/r * v +L/2/r * w                 
            
            T_Left  = r/2 * F - r/L * M
            T_Right = r/2 * F + r/L * M

            TV_Left  = r/2 * FV - r/L * MV
            TV_Right = r/2 * FV + r/L * MV

            ref_num = ref_num + 1            

            print(1)
            #print(v_fbk,w_fbk)
            #print(v_ref[ref_num],w_ref[ref_num])    
            print(s_ref[ref_num],t_ref[ref_num])  
            #print(s_fbk,t_fbk)    
            print(s_fbk-s_ref[ref_num], t_fbk-t_ref[ref_num])
            print(T_Left,T_Right)          
            #print(TV_Left,TV_Right)          
            #print(fbk.effort[0],-fbk.effort[1])          
            #print(fbk.position[0],fbk.position[1])
            #print(position[0],position[1])

            group_command.velocity = [0,0]                  
            group_command.effort = [T_Left,-T_Right]                  
    else:
        ref_num = 0
        s_ref[ref_num] = 0
        t_ref[ref_num] = 0
        iPos = fbk.position        
        #group_command.effort = [0.5,0.3]                         
    
    group.send_command(group_command)figure;plot(b(:,3))