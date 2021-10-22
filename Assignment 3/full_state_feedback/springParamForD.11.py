##Rita Abani 19244

##Dated : 22nd October 2021

''' This code pertains to a part of the parameters file required to tackle the D.11 question for the full state feedback system '''

import numpy as np
import control as cnt
import sys

import springParam as P ## I had saved a similar file for the previous question in D.8

Ts = P.Ts  # sample rate of the controller
beta = P.beta  # dirty derivative gain
force_max = P.force_max  # limit on control signal
m = P.m
ell = P.ell
g = P.g

#  now we focus on tuning parameters
tr = 1
zeta = 0.7

# Now we turn to State Space Equations
# xdot = A*x + B*u
# y = C*x

A = np.matrix([[0.0, 1.0],
               [-0.6, -0.1]])

B = np.matrix([[0.0],
               [0.2]])

C = np.matrix([[1.0, 0.0]])

# gain calculation
wn = 2.2/tr  # natural frequency
des_char_poly = [1, 2*zeta*wn, wn**2]
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 2:
    print("The system is not controllable")
else:
    K = cnt.acker(A, B, des_poles)
    kr = -1.0/(C[0]*np.linalg.inv(A-B*K)*B)

print('K: ', K)
print('kr: ', kr)
