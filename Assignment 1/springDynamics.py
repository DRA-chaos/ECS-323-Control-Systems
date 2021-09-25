

##Rita Abani 19244


import numpy as np 
import random
import springParam as P


class springDynamics:
    '''
        Model the physical system
    '''

    def __init__(self):
        # Initial state conditions
        self.state = np.matrix([[P.theta0],      # initial angle
                                [P.thetadot0]])  # initial angular rate
    
        alpha = 0.2  # Uncertainty parameter
        self.m = P.m * (1+2*alpha*np.random.rand()-alpha)  # Mass of the spring, kg
        self.b = P.b * (1+2*alpha*np.random.rand()-alpha)  # Damping coefficient, Ns
        self.k = P.k * (1+2*alpha*np.random.rand()-alpha)  # Spring constant

        self.Ts = P.Ts  # sample rate at which the dynamics are propagated

    def propagateDynamics(self, u):
        '''
            Integrate the differential equations defining dynamics
            P.Ts is the time step between function calls.
            u contains the system input(s).
        '''
        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = self.derivatives(self.state, u)
        k2 = self.derivatives(self.state + self.Ts/2*k1, u)
        k3 = self.derivatives(self.state + self.Ts/2*k2, u)
        k4 = self.derivatives(self.state + self.Ts*k3, u)
        self.state += self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)


    def outputs(self):
        '''
            Returns the measured outputs as a list
            [theta] with added Gaussian noise
        '''
        # re-label states for readability
        theta = self.state.item(0)
        # add Gaussian noise to outputs
        theta_m = theta + random.gauss(0, 0.001)
        # return measured outputs
        return [theta_m]

    def states(self):
        '''
            Returns all current states as a list
        '''
        return self.state.T.tolist()[0]






























    

   
