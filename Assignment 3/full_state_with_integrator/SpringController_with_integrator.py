##Rita Abani 19244

##Dated : 22nd October 2021

''' This code pertains to a part of the controller file required to tackle the D.11 question for the full state with integrator system'''


# The importnat terminologies have been commented along with the code
import numpy as np
import armParamHW12 as P

class armController:
    # state feedback control using dirty derivatives to estimate zdot
    def __init__(self):
        self.z_dot = 0.0          # derivative of z
        self.z_d1 = 0.0          # displacement z delayed by 1 sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # Input gain
        self.limit = P.force_max         # Maxiumum force
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]

        # differentiate z
        self.differentiatez(z)

        # integrate error
        error = z_r - z
        self.integrateError(error)

        # Construct the state
        x = np.matrix([[z], [self.z_dot]])

        # compute equilibrium torque force_e
        force_e = P.m * P.g * (P.ell / 2.0) * np.cos(z)

        # Compute the state feedback controller
        force_tilde = -self.K*x - self.ki*self.integrator

        # compute total torque
        force = self.saturate(force_e + force_tilde)
        return [force.item(0)]

    def differentiatez(self, z):
        '''
            differentiate z
        '''
        self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z - self.z_d1) / self.Ts)
        self.z_d1 = z

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
