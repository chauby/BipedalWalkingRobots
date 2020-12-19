#-*-coding:UTF-8 -*-
from math import sin
from math import cos

import numpy as np

# definitions of rotation matrix
def rotX(theta):
    matrix = np.matrix([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]])
    return matrix

def rotY(theta):
    matrix = np.matrix([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])
    return matrix

def rotZ(theta):
    matrix = np.matrix([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])
    return matrix

def matHat(vector):
    wx = vector[0]
    wy = vector[1]
    wz = vector[2]

    mat = np.zeros((3, 3))
    mat[0][0] = 0.
    mat[0][1] = -wz
    mat[0][2] = wy

    mat[1][0] = wz
    mat[1][1] = 0.
    mat[1][2] = -wx

    mat[2][0] = -wy
    mat[2][1] = wx
    mat[2][2] = 0.

    return mat

# Definition of Rodrigues function
def Rodrigues(a, q):
    output = np.eye(3) + matHat(a)*sin(q) + (1.0 - cos(q))*np.dot(matHat(a), matHat(a))
    return output
