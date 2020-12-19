#-*-coding:UTF-8 -*-
import numpy as np
from math import pi
from math import sqrt
from math import atan

import traceback

from utilities import Rodrigues

def jointLimitation(joint_angle, max_joint_angle, min_joint_angle):
    if joint_angle > max_joint_angle:
        joint_angle = max_joint_angle
    elif joint_angle < min_joint_angle:
        joint_angle = min_joint_angle
    
    return joint_angle

# Definition of link
class Link():
    def __init__(self, name, mother, sister, child, a, b, p, R, m, c, q=0, dq=0, dqq=0):
        self.name = name
        self.mother = mother
        self.sister = sister
        self.child = child 
        self.a = a 
        self.b = b 
        self.p = p
        self.R = R # rotation matrix
        self.m = m # mass
        self.c = c
        self.q = q # joint angle
        self.dq = dq # joint angular velocity
        self.dqq = dqq # joint angular accelearation

    def show(self):
        print('Link name:', self.name)
        print('mother:', self.mother)
        print('sister:', self.sister)
        print('child', self.child)
        print('a:', self.a)
        print('b:', self.b)
        print('p:', self.p)
        print('R:', self.R)
        print('m:', self.m)
        print('c:', self.c)
        print('q:', self.q)
        print('dq:', self.dq)
        print('dqq:', self.dqq)


# Deinition of LIPM kinetics model
class LIPM_KineticsModel():
    def __init__(self, name="LIPM", body=None, 
        L_LEG_J1=None, L_LEG_J2=None, L_LEG_J3=None, L_LEG_J4=None,
        R_LEG_J1=None, R_LEG_J2=None, R_LEG_J3=None, R_LEG_J4=None): 

        self.name = name
        self.mass = 0
        self.waist_width = 0.4 # distance between left hip and right hip
        self.thigh_length = 0.5
        self.shank_length = 0.5
        self.max_hip_joint_angle = pi/2  # revolute joint
        self.min_hip_joint_angle = -pi/2 # revolute joint
        self.max_knee_joint_angle = 0.5 # prismatic joint
        self.min_knee_joint_angle = -0.5 # prismatic joint

        self.ID = {'NONE': 0, 'BODY': 1, 'L_LEG_J1': 2, 'L_LEG_J2': 3, 'L_LEG_J3': 4, 'L_LEG_J4': 5, 'R_LEG_J1': 6, 'R_LEG_J2': 7, 'R_LEG_J3': 8, 'R_LEG_J4': 9}

        self.BODY= body
        self.L_LEG_J1 = L_LEG_J1
        self.L_LEG_J2 = L_LEG_J2
        self.L_LEG_J3 = L_LEG_J3
        self.L_LEG_J4 = L_LEG_J4
        self.R_LEG_J1 = R_LEG_J1
        self.R_LEG_J2 = R_LEG_J2
        self.R_LEG_J3 = R_LEG_J3
        self.R_LEG_J4 = R_LEG_J4

        # robot model's link group
        self.link_group = {
            self.ID.get('BODY'): self.BODY, 
            self.ID.get('L_LEG_J1'):self.L_LEG_J1,
            self.ID.get('L_LEG_J2'):self.L_LEG_J2,
            self.ID.get('L_LEG_J3'):self.L_LEG_J3,
            self.ID.get('L_LEG_J4'):self.L_LEG_J4,
            self.ID.get('R_LEG_J1'):self.R_LEG_J1,
            self.ID.get('R_LEG_J2'):self.R_LEG_J2,
            self.ID.get('R_LEG_J3'):self.R_LEG_J3,
            self.ID.get('R_LEG_J4'):self.R_LEG_J4}

    def show(self):
        self.BODY.show()
        print('------------------')
        self.L_LEG_J1.show()
        print('------------------')
        self.L_LEG_J2.show()
        print('------------------')
        self.L_LEG_J3.show()
        print('------------------')
        self.L_LEG_J4.show()
        print('------------------')
        self.R_LEG_J1.show()
        print('------------------')
        self.R_LEG_J2.show()
        print('------------------')
        self.R_LEG_J3.show()
        print('------------------')
        self.R_LEG_J4.show()

    def showPosition(self):
        print('BODY, Pos= ', self.BODY.p)
        print('L_LEG_J1, Pos= ', self.L_LEG_J1.p)
        print('L_LEG_J2, Pos= ', self.L_LEG_J2.p)
        print('L_LEG_J3, Pos= ', self.L_LEG_J3.p)
        print('L_LEG_J4, Pos= ', self.L_LEG_J4.p)
        print('R_LEG_J1, Pos= ', self.R_LEG_J1.p)
        print('R_LEG_J2, Pos= ', self.R_LEG_J2.p)
        print('R_LEG_J3, Pos= ', self.R_LEG_J3.p)
        print('R_LEG_J4, Pos= ', self.R_LEG_J4.p)
    
    def setJointAngle(self, joint_name, q):
        if joint_name == 'LEFT_HIP_AD':
            self.L_LEG_J1.q = q
        elif joint_name == 'LEFT_HIP_FE':
            self.L_LEG_J2.q = q
        elif joint_name == 'LEFT_KNEE':
            b = np.matrix((0, 0, -self.thigh_length + q)).T
            self.L_LEG_J3.b = b
        elif joint_name == 'RIGHT_HIP_AD':
            self.R_LEG_J1.q = q
        elif joint_name == 'RIGHT_HIP_FE':
            self.R_LEG_J2.q = q
        elif joint_name == 'RIGHT_KNEE':
            b = np.matrix((0, 0, -self.thigh_length + q)).T
            self.R_LEG_J3.b = b
        else:
            print('Error, can not recognize joint name: ', joint_name)


    def totalMass(self, j=1):
        if j == 0:
            m = 0
        else:
            link = self.link_group.get(j)
            m = link.m + self.totalMass(link.sister) + self.totalMass(link.child)
        self.mass = m
        return m

    def forwardKinematics(self, j=1):
        if j == 0:
            return

        link = self.link_group.get(j)
        if j != 1: # j=1 means BODY Link
            i = link.mother
            link_mother = self.link_group.get(i)
            link.p = np.dot(link_mother.R,  link.b) + link_mother.p
            link.R = np.dot(link_mother.R, Rodrigues(link.a, link.q))
        self.forwardKinematics(link.sister)
        self.forwardKinematics(link.child)

    def inverseKinematics(self, COM_pos, left_foot_pos, right_foot_pos, joint_limitation=False):
        left_leg_length = sqrt((COM_pos[0] - left_foot_pos[0])**2 + (COM_pos[2] - left_foot_pos[2])**2)
        tmp = float((left_foot_pos[0] - COM_pos[0])/(COM_pos[2] - left_foot_pos[2]))
        left_hip_q = atan(tmp)
        left_knee_q = self.thigh_length + self.shank_length - left_leg_length

        right_leg_length = sqrt((COM_pos[0] - right_foot_pos[0])**2 + (COM_pos[2] - right_foot_pos[2])**2)
        tmp = float((right_foot_pos[0] - COM_pos[0])/(COM_pos[2] - right_foot_pos[2]))
        right_hip_q = atan(tmp)
        right_knee_q = self.thigh_length + self.shank_length - right_leg_length

        if joint_limitation == True:
            left_hip_q = jointLimitation(left_hip_q, self.max_hip_joint_angle, self.min_hip_joint_angle)
            left_knee_q = jointLimitation(left_knee_q, self.max_knee_joint_angle, self.min_knee_joint_angle)
            right_hip_q = jointLimitation(right_hip_q, self.max_hip_joint_angle, self.min_hip_joint_angle)
            right_knee_q = jointLimitation(right_knee_q, self.max_knee_joint_angle, self.min_knee_joint_angle)

        q = [left_hip_q, left_knee_q, right_hip_q, right_knee_q]

        return q


# Create the LIPM's kinematics model
def createLIPMKinematicsModel():
    r = LIPM_KineticsModel()
    # ===============body================
    a = np.matrix((0, 0, 0)).T
    b = np.matrix((0, 0, 0)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 10
    c = np.matrix((0, 0, 2)).T
    link_body = Link("BODY", r.ID.get('NONE'), r.ID.get('NONE'), r.ID.get('L_LEG_J1'), a, b, p, R, m, c)

    # ===============left leg================
    a = np.matrix((1, 0, 0)).T
    b = np.matrix((0, r.waist_width/2, 0)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 0
    c = np.matrix((0, 0, 0)).T
    link_L_LEG_J1 = Link("L_LEG_J1", r.ID.get('BODY'), r.ID.get('R_LEG_J1'), r.ID.get('L_LEG_J2'), a, b, p, R, m, c)

    a = np.matrix((0, 1, 0)).T
    b = np.matrix((0, 0, 0)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 1
    c = np.matrix((0, 0, -0.25)).T
    link_L_LEG_J2 = Link("L_LEG_J2", r.ID.get('L_LEG_J1'), r.ID.get('NONE'), r.ID.get('L_LEG_J3'), a, b, p, R, m, c)

    a = np.matrix((0, 1, 0)).T
    b = np.matrix((0, 0, -r.thigh_length)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 1
    c = np.matrix((0, 0, -0.25)).T
    link_L_LEG_J3 = Link("L_LEG_J3", r.ID.get('L_LEG_J2'), r.ID.get('NONE'), r.ID.get('L_LEG_J4'), a, b, p, R, m, c)

    a = np.matrix((0, 1, 0)).T
    b = np.matrix((0, 0, -r.shank_length)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 0
    c = np.matrix((0, 0, -0.25)).T
    link_L_LEG_J4 = Link("L_LEG_J4", r.ID.get('L_LEG_J3'), r.ID.get('NONE'), r.ID.get('NONE'), a, b, p, R, m, c)


    #===============right leg================
    a = np.matrix((1, 0, 0)).T
    b = np.matrix((0, -r.waist_width/2, 0)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 0
    c = np.matrix((0, 0, 0)).T
    link_R_LEG_J1 = Link("R_LEG_J1", r.ID.get('BODY'), r.ID.get('NONE'), r.ID.get('R_LEG_J2'), a, b, p, R, m, c)

    a = np.matrix((0, 1, 0)).T
    b = np.matrix((0, 0, 0)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 1
    c = np.matrix((0, 0, -0.25)).T
    link_R_LEG_J2 = Link("R_LEG_J2", r.ID.get('R_LEG_J1'), r.ID.get('NONE'), r.ID.get('R_LEG_J3'), a, b, p, R, m, c)

    a = np.matrix((0, 1, 0)).T
    b = np.matrix((0, 0, -r.thigh_length)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 1
    c = np.matrix((0, 0, -0.25)).T
    link_R_LEG_J3 = Link("R_LEG_J3", r.ID.get('R_LEG_J2'), r.ID.get('NONE'), r.ID.get('R_LEG_J4'), a, b, p, R, m, c)

    a = np.matrix((0, 1, 0)).T
    b = np.matrix((0, 0, -r.shank_length)).T
    p = np.matrix((0, 0, 0)).T
    R = np.matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    m = 0
    c = np.matrix((0, 0, -0.25)).T
    link_R_LEG_J4 = Link("R_LEG_J4", r.ID.get('R_LEG_J3'), r.ID.get('NONE'), r.ID.get('NONE'), a, b, p, R, m, c)

    robot_model = LIPM_KineticsModel(body=link_body, 
        L_LEG_J1 = link_L_LEG_J1, L_LEG_J2 = link_L_LEG_J2, L_LEG_J3 = link_L_LEG_J3, L_LEG_J4 = link_L_LEG_J4, 
        R_LEG_J1 = link_R_LEG_J1, R_LEG_J2 = link_R_LEG_J2, R_LEG_J3 = link_R_LEG_J3, R_LEG_J4 = link_R_LEG_J4)

    return robot_model

if __name__ == "__main__":
    robot_model = createLIPMKinematicsModel()
    robot_model.show()

    robot_model.mass = robot_model.totalMass()
    print('Total mass:', robot_model.mass)

    pos = np.matrix((0, 0, 1)).T # set body's position
    robot_model.BODY.p = pos
    robot_model.forwardKinematics()
    robot_model.showPosition()

    print('------ joint angle test')
    q = pi/2
    robot_model.setJointAngle('LEFT_HIP_FE', q)
    q = -pi/2
    robot_model.setJointAngle('RIGHT_HIP_FE', q)

    q = 0.5
    robot_model.setJointAngle('LEFT_KNEE', q)
    q = -0.5
    robot_model.setJointAngle('RIGHT_KNEE', q)

    robot_model.forwardKinematics()
    robot_model.showPosition()
