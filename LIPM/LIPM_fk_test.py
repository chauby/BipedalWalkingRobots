#-*-coding:UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import sin
from math import pi

from LIPM_KinematicsModel import createLIPMKinematicsModel

# functions definition for animation
def initAnimation():
    LIPM_left_leg_ani.set_data([], [])
    LIPM_right_leg_ani.set_data([], [])
    LIPM_COM_ani.set_data(COM_x_array[0], COM_z_array[0])
    COM_pos.set_text('')

    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_COM_ani, COM_pos]

def animate(index):
    left_leg_x = [COM_x_array[index], left_foot_x_array[index]]
    left_leg_y = [COM_z_array[index], left_foot_z_array[index]]
    right_leg_x = [COM_x_array[index], right_foot_x_array[index]]
    right_leg_y = [COM_z_array[index], right_foot_z_array[index]]

    LIPM_left_leg_ani.set_data(left_leg_x, left_leg_y)
    LIPM_right_leg_ani.set_data(right_leg_x, right_leg_y)
    LIPM_COM_ani.set_data(COM_x_array[index], COM_z_array[index])
    COM_pos.set_text(COM_str % (COM_x_array[index], COM_z_array[index]))

    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_COM_ani, COM_pos]


robot_model = createLIPMKinematicsModel()
robot_model.show()

robot_model.mass = robot_model.totalMass()
print('Total mass:', robot_model.mass)

pos = np.matrix((0, 0, 0)).T
robot_model.BODY.p = pos
robot_model.forwardKinematics()
robot_model.showPosition()


COM_x_array = list()
COM_z_array = list()
left_foot_x_array = list()
left_foot_z_array = list()
right_foot_x_array = list()
right_foot_z_array = list()

time = 10
data_len = 500
delta_t = time / data_len

for t in np.linspace(0, time, data_len):
    left_hip_q = pi/2*sin(t)
    left_knee_q = 0.5*sin(t)
    right_hip_q = -pi/2*sin(t)
    right_knee_q = -0.5*sin(t)
    robot_model.setJointAngle('LEFT_HIP_FE', left_hip_q)
    robot_model.setJointAngle('LEFT_KNEE', left_knee_q)
    robot_model.setJointAngle('RIGHT_HIP_FE', right_hip_q)
    robot_model.setJointAngle('RIGHT_KNEE', right_knee_q)
    robot_model.forwardKinematics()

    COM_x = robot_model.BODY.p[0]
    COM_z = robot_model.BODY.p[2]
    left_foot_x = robot_model.L_LEG_J4.p[0]
    left_foot_z = robot_model.L_LEG_J4.p[2]
    right_foot_x = robot_model.R_LEG_J4.p[0]
    right_foot_z = robot_model.R_LEG_J4.p[2]

    # record the data
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
LIPM_COM_ani, = ax.plot(COM_x_array[0], COM_z_array[0], marker='o', markersize=20, color='r')

COM_str = 'COM = (%.1f, %.1f)'
COM_pos = ax.text(0.05, 0.9, '', transform=ax.transAxes)


ani_LIPM = animation.FuncAnimation(fig=fig, init_func=initAnimation, func=animate, frames=range(1, data_len), interval=1.0/delta_t, blit=True)
# ani_LIPM.save('LIPM_fk_test.gif', writer='imagemagick')

plt.show()
