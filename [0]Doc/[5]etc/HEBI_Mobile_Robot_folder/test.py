#!/usr/bin/env python3
import hebi
from time import sleep
import numpy as np
from numpy.distutils.fcompiler import none
import time
from math import pi
import math
import pygame
import xbox360_controller


# Mobile platform parameters
r = 0.1 #wheel radius
L = 0.4 #width btw axle
D = 0.3 #wheel base (distance btw actuators and caster)

# Simulation parameters
simulation_flag = 0
dt = 0.01
Vmax = 0.5
road_width = 0.7
circle_r = (road_width - L * math.cos(pi/4)) / (1-math.cos(pi/4))
dt = 0.01

print(circle_r)

# time for 1/4 circle travel
tf = round(pi *circle_r/(2*Vmax) +0.5,2)
t = np.linspace(0,tf,tf/dt+1)
#print(len(t))
Tnumber = np.linspace(0,1,21)
TRefa = -np.cos(Tnumber * pi)/2+0.5
TRefb = np.cos(Tnumber * pi)/2+0.5
SSRef = np.ones(len(t)-2*len(Tnumber))

Ref = np.concatenate( (np.concatenate((TRefa,SSRef)),TRefb) )
#print(Ref)
#print(len(Ref))
#print(Ref[1])
#v_ref = Vmax*(-m.cos(pi.*(t(1:1/dt)))/2+0.5 1*ones(1,length(t)-2/dt) + m.cos(pi.*(t(1:1/dt)))/2+0.5)


