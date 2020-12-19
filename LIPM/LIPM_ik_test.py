#-*-coding:UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import sin
from math import cos
from math import pi

from LIPM_KinematicsModel import createLIPMKinematicsModel

# functions definition for animation
def initAnimation():
    LIPM_left_leg_ani.set_data([], [])
    LIPM_right_leg_ani.set_data([], [])
    LIPM_COM_ani.set_data(COM_x_array[0], COM_z_array[0])
    COM_pos.set_text('')

    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_COM_ani, COM_pos]

left_foot_pos_x_array = list()
left_foot_pos_z_array = list()
right_foot_pos_x_array = list()
right_foot_pos_z_array = list()
def animate(index):
    left_leg_x = [COM_x_array[index], left_foot_x_array[index]]
    left_leg_y = [COM_z_array[index], left_foot_z_array[index]]
    right_leg_x = [COM_x_array[index], right_foot_x_array[index]]
    right_leg_y = [COM_z_array[index], right_foot_z_array[index]]

    LIPM_left_leg_ani.set_data(left_leg_x, left_leg_y)
    LIPM_right_leg_ani.set_data(right_leg_x, right_leg_y)

    left_foot_pos_x_array.append(left_foot_x_array[index])
    left_foot_pos_z_array.append(left_foot_z_array[index])
    right_foot_pos_x_array.append(right_foot_x_array[index])
    right_foot_pos_z_array.append(right_foot_z_array[index])

    LIPM_left_foot_ani.set_data(left_foot_pos_x_array, left_foot_pos_z_array)
    LIPM_right_foot_ani.set_data(right_foot_pos_x_array, right_foot_pos_z_array)

    LIPM_COM_ani.set_data(COM_x_array[index], COM_z_array[index])
    COM_pos.set_text(COM_str % (COM_x_array[index], COM_z_array[index]))

    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_left_foot_ani, LIPM_right_foot_ani, LIPM_COM_ani, COM_pos]


robot_model = createLIPMKinematicsModel()
robot_model.show()

robot_model.mass = robot_model.totalMass()
print('Total mass:', robot_model.mass)

pos = np.matrix((0, 0, 1)).T
robot_model.BODY.p = pos
robot_model.forwardKinematics()
robot_model.showPosition()

# # plot the 'heart' curve
# x = list()
# y = list()
# plt.figure()
# for t in np.linspace(0, 2*pi, 1000):
#     x.append(0.5*(sin(t)**3))
#     y.append(0.03*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t)))

# plt.plot(x, y, 'r')
# # plt.title('Heart')
# plt.axis('equal')
# plt.show()

COM_x_array = list()
COM_z_array = list()
left_foot_x_array = list()
left_foot_z_array = list()
right_foot_x_array = list()
right_foot_z_array = list()

data_len = 80
t1 = np.linspace(0, pi, data_len)
t2 = np.linspace(2*pi, pi, data_len)
delta_t = 0.05

loop = 5
for j in range(1, loop+1):
    for i in range(data_len):

        # # for single 'heart' curve
        # t = t1[i]
        # x1 = 0.4*(sin(t)**3)
        # z1 = 0.25 + 0.03*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))
        # t = t2[i]
        # x2 = 0.4*(sin(t)**3)
        # z2 = 0.25 + 0.03*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))

        # # for multiple 'heart' curves 1
        # t = t1[i]
        # x1 = 0.2*j*(sin(t)**3)
        # z1 = 0.25 + 0.02*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))
        # t = t2[i]
        # x2 = 0.2*j*(sin(t)**3)
        # z2 = 0.25 + 0.02*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))

        # # for multiple 'heart' curves 2
        t = t1[i]
        x1 = 0.15*j*(sin(t)**3)
        z1 = 0.25 + 0.01*j*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))
        t = t2[i]
        x2 = 0.15*j*(sin(t)**3)
        z2 = 0.25 + 0.01*j*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))


        # calculate the desired left foot position and desired right foot position
        left_foot_target_pos = [x1, robot_model.L_LEG_J4.p[1], z1]
        right_foot_target_pos = [x2, robot_model.R_LEG_J4.p[1], z2]
        q = robot_model.inverseKinematics(robot_model.BODY.p, left_foot_target_pos, right_foot_target_pos, joint_limitation=False)

        # set joint angles
        robot_model.setJointAngle('LEFT_HIP_FE', q[0])
        robot_model.setJointAngle('LEFT_KNEE', q[1])
        robot_model.setJointAngle('RIGHT_HIP_FE', q[2])
        robot_model.setJointAngle('RIGHT_KNEE', q[3])
        robot_model.forwardKinematics()

        # record data
        COM_x = robot_model.BODY.p[0]
        COM_z = robot_model.BODY.p[2]
        left_foot_x = robot_model.L_LEG_J4.p[0]
        left_foot_z = robot_model.L_LEG_J4.p[2]
        right_foot_x = robot_model.R_LEG_J4.p[0]
        right_foot_z = robot_model.R_LEG_J4.p[2]

        COM_x_array.append(COM_x)
        COM_z_array.append(COM_z)
        left_foot_x_array.append(left_foot_x)
        left_foot_z_array.append(left_foot_z)
        right_foot_x_array.append(right_foot_x)
        right_foot_z_array.append(right_foot_z)


# plot the animation
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-1.5, 1.5))
# ax.set_aspect('equal')
ax.grid(ls='--')

LIPM_left_leg_ani, = ax.plot([], [], 'o-', lw=4, color='b')
LIPM_right_leg_ani, = ax.plot([], [], 'o-', lw=4, color='k')
LIPM_left_foot_ani, = ax.plot(left_foot_x_array[0], left_foot_z_array[0], '.', markersize=2, color='m')
LIPM_right_foot_ani, = ax.plot(right_foot_x_array[0], right_foot_z_array[0], '.', markersize=2, color='r')
LIPM_COM_ani, = ax.plot(COM_x_array[0], COM_z_array[0], marker='o', markersize=20, color='r')

COM_str = 'COM = (%.1f, %.1f)'
COM_pos = ax.text(0.05, 0.9, '', transform=ax.transAxes)

ani_LIPM = animation.FuncAnimation(fig=fig, init_func=initAnimation, func=animate, frames=range(1, data_len*loop), interval=1.0/delta_t, blit=True)
# ani_LIPM.save('LIPM_ik_test.gif', writer='pillow')

plt.show()
