
#Rita Abani 19244
# Code for Design Study Assignment 4 , Date: 09/11/2021


import numpy as np
import springParamHW13 as P

class springController:
    # state feedback control using dirty derivatives to estimate thetadot
    def __init__(self):
        self.x_hat = np.matrix([
            [0.0],
            [0.0],
        ])
        self.force_d1 = 0.0            # control torque, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # Input gain
        self.L = P.L                 # observer gain
        self.A = P.A                 # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.force_max         # Maxiumum force
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        theta_r = y_r[0]
        theta = y[0]

        # update the observer and extract theta_hat
        self.updateObserver(y)
        theta_hat = self.x_hat[0]

        # integrate error
        error = theta_r - theta
        self.integrateError(error)

        # compute equilibrium torque force_e
        theta_hat = self.x_hat[0]
        force_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)

        # Compute the state feedback controller
        force_tilde = -self.K*self.x_hat - self.ki*self.integrator

        # compute total torque
        force = self.saturate(force_e + force_tilde)
        self.updateTorque(force)
        return [force.item(0)]

    def updateObserver(self, y_m):
        # compute equilibrium torque force_e
        theta_hat = self.x_hat[0]
        force_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)

        N = 10
        for i in range(0, N):
            self.x_hat = self.x_hat + self.Ts/float(N)*(
                self.A*self.x_hat
                + self.B*(self.force_d1 - force_e)
                + self.L*(y_m-self.C*self.x_hat)
            )

    def updateTorque(self, force):
        self.force_d1 = force

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
