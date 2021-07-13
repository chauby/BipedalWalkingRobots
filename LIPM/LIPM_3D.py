# -*-coding:UTF-8 -*-
import numpy as np

# Deinition of 3D Linear Inverted Pendulum
class LIPM3D:
    def __init__(self,
                 dt=0.001,
                 T_sup=1.0,
                 support_leg='left_leg'):
        self.dt = dt
        self.t = 0
        self.T_sup = T_sup # support time

        self.p_x = 0  # desired foot location x
        self.p_y = 0  # desired foot location y

        self.p_x_star = 0 # modified foot location x
        self.p_y_star = 0 # modified foot location y

        # Initialize the gait parameters
        self.s_x = 0.0
        self.s_y = 0.0

        # COM initial state
        self.x_0 = 0
        self.vx_0 = 0
        self.y_0 = 0
        self.vy_0 = 0

        # COM real-time state
        self.x_t = 0
        self.vx_t = 0
        self.y_t = 0
        self.vy_t = 0

        # COM desired state
        self.x_d = 0
        self.vx_d = 0
        self.y_d = 0
        self.vy_d = 0

        # final state for one gait unit
        self.bar_x = 0.0
        self.bar_y = 0.0
        self.bar_vx = 0.0
        self.bar_vy = 0.0

        self.support_leg = support_leg
        self.left_foot_pos = [0.0, 0.0, 0.0]
        self.right_foot_pos = [0.0, 0.0, 0.0]
        self.COM_pos = [0.0, 0.0, 0.0]
    
    def initializeModel(self, COM_pos, left_foot_pos, right_foot_pos):
        self.COM_pos = COM_pos
        self.left_foot_pos = left_foot_pos
        self.right_foot_pos = right_foot_pos

        self.zc = self.COM_pos[2]
        self.T_c = np.sqrt(self.zc / 9.81) # set gravity parameter as 9.81
        self.C = np.cosh(self.T_sup/self.T_c)
        self.S = np.sinh(self.T_sup/self.T_c)
    
    def updateParameters(self, T_sup):
        self.T_sup = T_sup
        self.C = np.cosh(self.T_sup/self.T_c)
        self.S = np.sinh(self.T_sup/self.T_c)

    def step(self):
        self.t += self.dt
        t = self.t
        T_c = self.T_c

        self.x_t = self.x_0*np.cosh(t/T_c) + T_c*self.vx_0*np.sinh(t/T_c)
        self.vx_t = self.x_0/T_c*np.sinh(t/T_c) + self.vx_0*np.cosh(t/T_c)

        self.y_t = self.y_0*np.cosh(t/T_c) + T_c*self.vy_0*np.sinh(t/T_c)
        self.vy_t = self.y_0/T_c*np.sinh(t/T_c) + self.vy_0*np.cosh(t/T_c)

    def calculateXtVt(self, t):
        T_c = self.T_c

        x_t = self.x_0*np.cosh(t/T_c) + T_c*self.vx_0*np.sinh(t/T_c)
        vx_t = self.x_0/T_c*np.sinh(t/T_c) + self.vx_0*np.cosh(t/T_c)

        y_t = self.y_0*np.cosh(t/T_c) + T_c*self.vy_0*np.sinh(t/T_c)
        vy_t = self.y_0/T_c*np.sinh(t/T_c) + self.vy_0*np.cosh(t/T_c)

        return x_t, vx_t, y_t, vy_t

    def nextReferenceFootLocation(self, s_x, s_y, theta=0):
        if self.support_leg is 'left_leg': # then the next support leg is the right leg
            p_x_new = self.p_x + np.cos(theta)*s_x - np.sin(theta)*s_y
            p_y_new = self.p_y + np.sin(theta)*s_x + np.cos(theta)*s_y
        elif self.support_leg is 'right_leg': # then the next support leg is the left leg
            p_x_new = self.p_x + np.cos(theta)*s_x + np.sin(theta)*s_y
            p_y_new = self.p_y + np.sin(theta)*s_x - np.cos(theta)*s_y

        return p_x_new, p_y_new

    def nextState(self, s_x, s_y, theta=0):
        '''
        Calculate next final state at T_sup
        '''
        if self.support_leg is 'left_leg':
            bar_x_new = np.cos(theta)*s_x/2.0 - np.sin(theta)*s_y/2.0
            bar_y_new = np.sin(theta)*s_x/2.0 + np.cos(theta)*s_y/2.0
        elif self.support_leg is 'right_leg':
            bar_x_new = np.cos(theta)*s_x/2.0 + np.sin(theta)*s_y/2.0
            bar_y_new = np.sin(theta)*s_x/2.0 - np.cos(theta)*s_y/2.0
        return bar_x_new, bar_y_new

    def nextVel(self, bar_x=0, bar_y=0, theta=0):
        C = self.C
        S = self.S
        T_c = self.T_c

        bar_vx_new = np.cos(theta)*(1+C)/(T_c*S)*bar_x - np.sin(theta)*(C-1)/(T_c*S)*bar_y
        bar_vy_new = np.sin(theta)*(1+C)/(T_c*S)*bar_x + np.cos(theta)*(C-1)/(T_c*S)*bar_y

        return bar_vx_new, bar_vy_new

    def targetState(self, p_x, bar_x, bar_vx):
        x_d = p_x + bar_x
        vx_d = bar_vx

        return x_d, vx_d
    
    def modifiedFootLocation(self, a=1.0, b=1.0, x_d=0, vx_d=0, x_0=0, vx_0=0):
        C = self.C
        S = self.S
        T_c = self.T_c
        D = a*(C - 1)**2 + b*(S/T_c)**2

        p_x_star = -a*(C-1)*(x_d - C*x_0 - T_c*S*vx_0)/D - b*S*(vx_d - S*x_0/T_c - C*vx_0)/(T_c*D)

        return p_x_star

    def calculateFootLocationForNextStep(self, s_x=0.0, s_y=0.0, a=1.0, b=1.0, theta=0.0, x_0=0.0, vx_0=0.0, y_0=0.0, vy_0=0.0):
        self.s_x = s_x
        self.s_y = s_y

        # ----------------------------- calculate desired COM states and foot locations for the given s_x, s_y and theta
        # calculate desired foot locations
        print(self.p_x, self.p_y)
        p_x_new, p_y_new = self.nextReferenceFootLocation(s_x, s_y, theta)
        # print('-- p_x_new=%.3f'%p_x_new, ', p_y_new=%.3f'%p_y_new)

        # calculate desired COM states
        bar_x, bar_y = self.nextState(s_x, s_y, theta)
        bar_vx, bar_vy = self.nextVel(bar_x, bar_y, theta)
        # print('-- bar_x=%.3f'%bar_x, ', bar_y=%.3f'%bar_y)
        # print('-- bar_vx=%.3f'%bar_vx, ', bar_vy=%.3f'%bar_vy)

        # calculate target COM state in the next step
        self.x_d, self.vx_d = self.targetState(p_x_new, bar_x, bar_vx)
        self.y_d, self.vy_d = self.targetState(p_y_new, bar_y, bar_vy)
        # print('-- x_d=%.3f'%self.x_d, ', vx_d=%.3f'%self.vx_d)
        # print('-- y_d=%.3f'%self.y_d, ', vy_d=%.3f'%self.vy_d)

        # ----------------------------- calculate modified foot locations based on the current actual COM states
        # correct the modified foot locations to minimize the errors
        self.p_x_star = self.modifiedFootLocation(a, b, self.x_d, self.vx_d, x_0, vx_0)
        self.p_y_star = self.modifiedFootLocation(a, b, self.y_d, self.vy_d, y_0, vy_0)
        # print('-- p_x_star=%.3f'%self.p_x_star, ', p_y_star=%.3f'%self.p_y_star)

    def switchSupportLeg(self):
        if self.support_leg is 'left_leg':
            print('\n---- switch the support leg to the right leg')
            self.support_leg = 'right_leg'
            COM_pos_x = self.x_t + self.left_foot_pos[0]
            COM_pos_y = self.y_t + self.left_foot_pos[1]
            self.x_0 = COM_pos_x - self.right_foot_pos[0]
            self.y_0 = COM_pos_y - self.right_foot_pos[1]
        elif self.support_leg is 'right_leg':
            print('\n---- switch the support leg to the left leg')
            self.support_leg = 'left_leg'
            COM_pos_x = self.x_t + self.right_foot_pos[0]
            COM_pos_y = self.y_t + self.right_foot_pos[1]
            self.x_0 = COM_pos_x - self.left_foot_pos[0]
            self.y_0 = COM_pos_y - self.left_foot_pos[1]

        self.t = 0
        self.vx_0 = self.vx_t
        self.vy_0 = self.vy_t
