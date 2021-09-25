
##Rita Abani 19244, solution for Design Assignment D.3 (e)

import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory to Rita PyCharm

import springParam as P
from signalGenerator import signalGenerator
from springAnimation import springAnimation
from plotData import plotData
from springDynamics import springDynamics
from damper_input import *

# we are instantiating spring mass parameters and reference classes
spring = springDynamics()
reference = signalGenerator(amplitude=0.01, frequency=0.02)

# instantiate the simulation plots and animation
#dataPlot = plotData()
animation = springAnimation()

t = P.t_start  # time starts at t_start
obj = mySlider()
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        #tau=torque.square(t)
        tau = obj.getValue()
        print(tau)
        obj.update(tau)
        spring.propagateDynamics(tau)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawspring(spring.states())
    #dataPlot.updatePlots(t, ref_input, spring.states(), tau)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation


