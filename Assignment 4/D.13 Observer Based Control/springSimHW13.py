
#Rita Abani 19244
# Code for Design Study Assignment 4 , Date: 09/11/2021


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
from plotObserverData import plotObserverData


# instantiate spring, controller, and reference classes
spring = springDynamics()
ctrl = springController()
reference = signalGenerator(amplitude=30*np.pi/180.0, frequency=0.05)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = springAnimation()
observerPlot = plotObserverData()

# set disturbance input
disturbance = 0.01

p = []

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
    #dataPlot.updatePlots(t, ref_input, spring.states(), u)
    #observerPlot.updatePlots(t, spring.states(), ctrl.x_hat)
    c = np.asarray(ctrl.x_hat[0][0])
    p.append((spring.states()[0] - c[0][0]))
    print (spring.states()[0], c[0][0])
    #plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

print(p)
plt.plot(p)
plt.savefig('./yo.jpg')

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
