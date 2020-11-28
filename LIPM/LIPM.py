#-*-coding:UTF-8 -*-
from math import sqrt
from math import sinh
from math import cosh
import numpy as np


class LIPM:
    def __init__(self, x0 = 0, v0 = 0, z0 = 1, delta_t = 0.001):
        self.x0 = x0
        self.v0 = v0 
        self.z0 = z0

        self.x = x0
        self.zc = z0 # z constant

        self.delta_t = delta_t
        self.xt = 0
        self.zt = 0
        self.vt = 0
        self.g = 9.8 # gravity acceleration constant
        self.mass = 10.0
        self.acc = 0.0
        self.Tc = sqrt(self.zc / self.g)
        self.t = 0

        self.left_foot_x = 0
        self.left_foot_z = 0
        self.right_foot_x = 0
        self.right_foot_z = 0
        self.support_leg = 'left_leg' # left_leg or right_leg

        self.orbital_energy = 0
        self.target_orbital_energy = 0
        self.capture_point = 0

        self.step_length = 0
        self.xf = 0
        self.vf = 0

    def update(self):
        self.t += self.delta_t
        self.xt, self.vt = self.updateXtVt(self.x0, self.v0, self.t)
        self.zt = self.zc
        self.orbital_energy = self.updateOrbitalEnergy(self.xt, self.vt)

    # update xt and vt at time t
    def updateXtVt(self, x0, v0, t):
        tau = t/self.Tc
        xt = x0*cosh(tau) + self.Tc*v0*sinh(tau)
        vt = (x0/self.Tc)*sinh(tau) + v0*cosh(tau)
        return xt, vt

    # update the orbital energy, for each single cycle, the orbital energy keeps constant
    def updateOrbitalEnergy(self, x, v):
        orbital_energy = (v**2 - (self.g/(self.zc))*x**2)/2
        return orbital_energy

    # update the capture point associated with the target orbital energy
    def updateCapturePoint(self, xt, vt, target_OE):
        if(vt**2 < 2*target_OE):
            capture_point = self.capture_point
            print("warning: calculate the capture point failed, the velocity is too low. caputre_point=", capture_point)
        else:
            capture_point = xt + np.sign(vt)*sqrt((vt**2 - 2*target_OE)*self.zc/self.g)
        return capture_point

    # switch the support leg for continue walking
    def switchSupportLeg(self):
        if self.support_leg == 'left_leg':
            self.x0 = self.left_foot_x + self.xt - self.right_foot_x
            self.support_leg = 'right_leg'
        elif self.support_leg == 'right_leg':
            self.x0 = self.right_foot_x + self.xt - self.left_foot_x
            self.support_leg = 'left_leg'
        else:
            print('Error: support leg is a wrong value:', self.support_leg)

        print("t = %.2f" % self.t + ", switch the support leg to " + self.support_leg)
        self.t = 0 # reset t
        self.v0 = self.vt

if __name__ == '__main__':
    x0 = 0.0
    v0 = 2.0
    z0 = 1.0
    delta_t = 0.01
    LIPM_model = LIPM(x0, v0, z0, delta_t) # x0, v0, z, delta_t
    LIPM_model.target_orbital_energy = 0.0
