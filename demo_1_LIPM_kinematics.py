#-*-coding:UTF-8 -*-
import sys
import time
from math import sin
from math import cos
from math import pi
import numpy as np


sys.path.append('./VREP_remoteAPIs')
import sim as vrep_sim

from LIPMSimModel import LIPMSimModel

sys.path.append('./LIPM')
from LIPM_KinematicsModel import createLIPMKinematicsModel

print ('Program started')
vrep_sim.simxFinish(-1) # just in case, close all opened connections
client_ID = vrep_sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5) # Connect to CoppeliaSim
LIPM_sim_model = LIPMSimModel()
LIPM_sim_model.initializeSimModel(client_ID)


robot_model = createLIPMKinematicsModel()
robot_model.show()

robot_model.mass = robot_model.totalMass()
print('Total mass:', robot_model.mass)

pos = np.matrix((0, 0, 1)).T
robot_model.BODY.p = pos
robot_model.forwardKinematics()
robot_model.showPosition()

''' Main control loop '''
print('begin main control loop ...')
data_len = 1000
t1 = np.linspace(0, pi, data_len)
t2 = np.linspace(2*pi, pi, data_len)
loop = 5
for j in range(1, loop+1):
    for i in range(data_len):
        # for multiple 'heart' curves
        t = t1[i]
        x1 = j*(sin(t)**3)
        z1 = j*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))
        
        t = t2[i]
        x2 = j*(sin(t)**3)
        z2 = j*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))

        # scale and move the curve to fit the kinematics of LIPM
        x1 = 0.06*x1
        x2 = 0.06*x2
        z1 = -0.05 + 0.004*z1
        z2 = -0.05 + 0.004*z2

        # calculate the desired left foot position and desired right foot position
        left_foot_target_pos = [x1, robot_model.L_LEG_J4.p[1], z1]
        right_foot_target_pos = [x2, robot_model.R_LEG_J4.p[1], z2]
        q = robot_model.inverseKinematics(robot_model.BODY.p, left_foot_target_pos, right_foot_target_pos, joint_limitation=False)

        # set joint angles
        q[2] = -q[2]
        LIPM_sim_model.setJointAngle(q)
        time.sleep(1.0/data_len)
