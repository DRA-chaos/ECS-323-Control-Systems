

##Rita Abani 19244


import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np




class plotData:
    k1 = 2.989  # cart 1 spring constant (N/m)

    b1 = 0.495  # cart 1 viscous damping coefficient (kg/s)

    m1 = 4.922  # cart 1 mass (kg)

    x10 = 1  # cart 1 initial position (m)

    v10 = 0  # cart 1 initial velocity (m/s)

    # Time step
    simTime = 10  # simulation time (s)
    tStep = 0.001  # simulation time step
    iterations = int(simTime / tStep)  # total number of iterations
    t = np.arange(0, iterations)

    # We add initial conditions and also allocate variables for speed
    x1 = np.zeros((iterations, 1))
    x1[0, :] = x10

    v1 = np.zeros((iterations, 1))
    v1[0, :] = v10

    a1 = np.zeros((iterations, 1))
    a1[0, :] = -(b1 * v10  + k1 * x10 ) / m1

    # Solve the ODE's with Euler's Method
    for n in range(1, iterations):
        
        x1[n, :] = x1[n - 1, :] + v1[n - 1, :] * tStep  # cart 1 position

        v1[n, :] = v1[n - 1, :] + a1[n - 1, :] * tStep  # cart 1 velocity

    # Find mass accelerations
        a1[n, :] = -(b1 * v1[n, :]  + k1 * x1[n, :] ) / m1

# Plot results
plt.rcParams["figure.figsize"] = (15, 10)  # resizes figures for viewing
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t, x1, 'r', label='Cart 1')

plt.ylabel('Position (m)')
plt.title('Position as a Function of Time')
plt.legend()
#plt.subplot(3, 1, 2)
#plt.plot(t, v1, 'b', label='Spring Mass Damper System')































    

   
