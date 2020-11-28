#-*-coding:UTF-8 -*-
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from LIPM import LIPM

# functions definition for animation
def initAnimation():
    LIPM_left_leg_ani.set_data([], [])
    LIPM_right_leg_ani.set_data([], [])
    LIPM_COM_ani.set_data(COM_x[0], COM_z[0])
    COM_pos.set_text('')
    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_COM_ani, COM_pos]

def animate(i):
    left_leg_x = [COM_x[i], left_foot_x_array[i]]
    left_leg_y = [COM_z[i], left_foot_z_array[i]]
    right_leg_x = [COM_x[i], right_foot_x_array[i]]
    right_leg_y = [COM_z[i], right_foot_z_array[i]]

    LIPM_left_leg_ani.set_data(left_leg_x, left_leg_y)
    LIPM_right_leg_ani.set_data(right_leg_x, right_leg_y)
    LIPM_COM_ani.set_data(COM_x[i], COM_z[i])
    COM_pos.set_text(COM_str % (COM_x[i], COM_z[i]))
    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_COM_ani, COM_pos]


x0 = -1.0
v0 = 3.5
z0 = 1.0
delta_t = 0.02
LIPM_model = LIPM(x0, v0, z0, delta_t) # x0, v0, z, delta_t

data_len = 60
orbital_energy_array = list()
xt_array = list()
vt_array = list()
COM_x = list()
COM_z = list()
left_foot_x_array = list()
left_foot_z_array = list()
right_foot_x_array = list()
right_foot_z_array = list()

for i in range(data_len):
    LIPM_model.update()

    xt_array.append(LIPM_model.xt)
    vt_array.append(LIPM_model.vt)
    COM_x.append(LIPM_model.xt)
    COM_z.append(LIPM_model.zt)
    left_foot_x_array.append(LIPM_model.left_foot_x)
    left_foot_z_array.append(LIPM_model.left_foot_z)
    right_foot_x_array.append(LIPM_model.right_foot_x)
    right_foot_z_array.append(LIPM_model.right_foot_z)

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-0.2, 1.2))
# ax.set_aspect('equal')
ax.grid(ls='--')

LIPM_left_leg_ani, = ax.plot([], [], 'o-', lw=4, color='b')
LIPM_right_leg_ani, = ax.plot([], [], 'o-', lw=4, color='k')
LIPM_COM_ani, = ax.plot(COM_x[0], COM_z[0], marker='o', markersize=20, color='r')

COM_str = 'COM = (%.1f, %.1f)'
COM_pos = ax.text(0.05, 0.9, '', transform=ax.transAxes)

ani = animation.FuncAnimation(fig=fig, init_func=initAnimation, func=animate, frames=range(1, data_len), interval=1.0/delta_t, blit=True)
# ani.save('LIPM_single_leg.gif', writer='imagemagick') # save the animation to files
plt.show()
