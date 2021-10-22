##Rita Abani 19244

##Dated : 22nd October 2021

''' This code pertains to a part of the parameters file required to tackle the D.11 question for the full state feedback system '''


import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import springParam as P
from springDynamics import springDynamics
from springController import springController
from signalGenerator import signalGenerator
from springAnimation import springAnimation
from plotData import plotData

# instantiate spring, controller, and reference classes
spring = springDynamics()
ctrl = springController()
reference = signalGenerator(amplitude=30*np.pi/180.0, frequency=0.05)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = springAnimation()

# set disturbance input
disturbance = 0.01

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, spring.outputs())  # Calculate the control value
        sys_input = [u[0]+ disturbance]
        spring.propagateDynamics(sys_input)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawspring(spring.states())
    dataPlot.updatePlots(t, ref_input, spring.states(), u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation


#Note: Reference book used : Introduction to Feedback Control using Design Studies
