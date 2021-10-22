##RITA ABANI 19244
##DATE: 22-10-21
''' THE FOLLOWING IS THE CODE PERTAINING TO A PART OF THE DESIGN STUDY ASSIGNMENT 3 , D.11 '''


import numpy as np
import pARAMETERS FOR d.10 as P

class springController:
    # here we are using state feedback control using dirty derivatives to estimate zdot
    def __init__(self):
        self.z_dot = 0.0          # derivative of z
        self.z_d1 = 0.0          # Angle z delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.kr = P.kr               # Input gain
        self.limit = P.force_max         # Maxiumum force
        self.beta = P.beta           
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]

        # differentiate z
        self.differentiatez(z)

        # Construct the state
        x = np.matrix([[z], [self.z_dot]])

        # compute equilibrium  force_e
        force_e = P.m * P.g * (P.ell / 2.0) * np.cos(z)

        # Compute the state feedback controller
        force_tilde = -self.K*x + self.kr*z_r

        # compute total force
        force = self.saturate(force_e + force_tilde)
        return [force.item(0)]

    def differentiatez(self, z):
        '''
            differentiate z
        '''
        self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z - self.z_d1) / self.Ts)
        self.z_d1 = z

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
