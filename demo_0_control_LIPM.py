#-*-coding:UTF-8 -*-
import math
import sys
import time

sys.path.append('./VREP_remoteAPIs')
import sim as vrep_sim

from LIPMSimModel import LIPMSimModel

if __name__ == '__main__':
    print ('Program started')
    vrep_sim.simxFinish(-1) # just in case, close all opened connections
    client_ID = vrep_sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5) # Connect to CoppeliaSim
    LIPM_sim_model = LIPMSimModel()
    LIPM_sim_model.initializeSimModel(client_ID)


    ''' Main control loop '''
    print('begin main control loop ...')
    t = 0
    while True:
        # Motion planning
        t = t + 0.01
        hip_disred_angle = math.sin(t)
        knee_disred_angle = 0.25*math.cos(t)
        q = [hip_disred_angle, knee_disred_angle, hip_disred_angle, knee_disred_angle]

        LIPM_sim_model.setJointAngle(q)
        time.sleep(0.01)
