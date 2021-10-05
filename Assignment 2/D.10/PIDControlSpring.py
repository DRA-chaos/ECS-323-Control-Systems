# Rita Abani 19244

#implementing PID control for part a and b of D10

import numpy as np

class PIDControl:
    def __init__(self, kp, ki, kd, limit, beta, Ts):
        self.kp = kp                 # Proportional control gain
        self.ki = ki                 # Integral control gain
        self.kd = kd                 # Derivative control gain
        self.limit = limit           # The output will saturate at this limit
        self.beta = beta             # gain for dirty derivative
        self.Ts = Ts                 # sample rate

        self.y_dot = 0.0             # estimated derivative of y
        self.y_d1 = 0.0              # Signal y delayed by one sample
        self.error_dot = 0.0         # estimated derivative of error
        self.error_d1 = 0.0          # Error delayed by one sample
        self.integrator = 0.0        # integrator

    def PID(self, y_r, y, flag=True):
        '''
            PID control,
            if flag==True, then returns
                u = kp*error + ki*integral(error) + kd*error_dot.
            else returns 
                u = kp*error + ki*integral(error) - kd*y_dot.
            error_dot and y_dot are computed numerically using a dirty derivative
            integral(error) is computed numerically using trapezoidal approximation
        '''

        # Compute the current error
        error = y_r - y
        # integrate error
        self.integrator = self.integrator + (self.Ts/2)*(error+self.error_d1)

        # PID Control
        if flag is True:
            # differentiate error
            self.error_dot = self.beta * self.error_dot \
                             + (1 - self.beta) * ((error - self.error_d1) / self.Ts)
            # PID control
            u_unsat = self.kp*error + self.ki*self.integrator + self.kd*self.error_dot
        else:
            # differentiate y
            self.y_dot = self.beta * self.y_dot \
                             + (1 - self.beta) * ((y - self.y_d1) / self.Ts)
            # PID control
            u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.y_dot
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator + self.Ts / self.ki * (u_sat - u_unsat)
        # update delayed variables
        self.error_d1 = error
        self.y_d1 = y
        return u_sat

    def PD(self, y_r, y, flag=True):
        '''
            PD control,
            
            if flag==True, then returns
                u = kp*error + kd*error_dot.
            else returns 
                u = kp*error - kd*y_dot.
            
            error_dot and y_dot are computed numerically using a dirty derivative
        '''

        # Compute the current error
        error = y_r - y

        # PD Control
        if flag is True:
            # differentiate error
            self.error_dot = self.beta * self.error_dot \
                             + (1 - self.beta) * ((error - self.error_d1) / self.Ts)
            # PD control
            u_unsat = self.kp*error + self.kd*self.error_dot
        else:
            # differentiate y
            self.y_dot = self.beta * self.y_dot \
                             + (1 - self.beta) * ((y - self.y_d1) / self.Ts)
            # PD control
            u_unsat = self.kp*error - self.kd*self.y_dot
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        # update delayed variables
        self.error_d1 = error
        self.y_d1 = y
        return u_sat

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

        # create rectangle on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the Rectangle patch and append its handle to the handle list
            self.handle.append(
                mpatches.Rectangle(corner, P.w, P.h,
                                   fc = 'cyan', ec = 'black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[0])
        else:
            self.handle[0].set_xy(corner)  # Update patch

    def draw_spring(self,z):
        def spring(start, end, node, width):
            # Check that value of node is atleast 1
            node = max(int(node), 1)

            # Convert start and end to numpy arrays for inputs of different shapes or types
            start, end = np.array(start).reshape((2,)), np.array(end).reshape((2,))

            # If both points coincide, return value of x,y
            if (start == end).all():
                return start[0], start[1]

            # length of spring 
            length = np.linalg.norm(np.subtract(end, start))

            # unit vectors tangent (u_t) and normal (u_t) to spring
            u_t = np.subtract(end,start)/length
            u_n = np.array([[0,-1],[1,0]]).dot(u_t)

            # Initializing an array of x and y coordinates of node points +2 points where x is the 0th row and y is the 1st row of the array
            coordinates = np.zeros((2,node + 2))
            coordinates[:,0],coordinates[:,-1] = start,end

            # length should not greater be than the total length the spring
            # computing the normal distance from the centerline of the spring.
            normal_dist = math.sqrt(max(0, width**2 - (length**2/node**2)))/2

            # Compute the coordinates of each node point
            for i in range(1,node+1):
                coordinates[:,i] = (start + ((length*(2*i - 1)*u_t)/(2*node)) + (normal_dist*(-1)**i*u_n))
            return coordinates[0,:], coordinates[1,:]

        x,y = spring((P.wall,P.h*0.5),(z,P.h*0.5),20,0.4)

        # creating spring on first call and updating it on each call after it
        if self.flag_init == True:
            # Creating object for spring and appending its handle to handle list
            spring, =self.ax.plot(x,y,lw=1,c='blue')
            self.handle.append(spring)
        else:
            self.handle[1].set_xdata(x)
            self.handle[1].set_ydata(y)

    def draw_damper(self,z):
        # Function for drawing damper
        x1,y1 = (P.wall,(P.wall+z)/2),(P.h*0.25,P.h*0.25)
        x2,y2 = ((P.wall+z)/2,(P.wall+z)/2),(P.h*0.25-P.h*0.1,P.h*0.25+P.h*0.1)
        x3,y3 = ((P.wall+z)/2,(P.wall+z)/2+0.1),(P.h*0.25+P.h*0.1,P.h*0.25+P.h*0.1)
        x4,y4 = ((P.wall+z)/2,(P.wall+z)/2+0.1),(P.h*0.25-P.h*0.1,P.h*0.25-P.h*0.1)
        x5,y5 = ((P.wall+z)/2+0.1,(P.wall+z)/2+0.1),(P.h*0.25-P.h*0.05,P.h*0.25+P.h*0.05)
        x6,y6 = ((P.wall+z)/2,z),(P.h*0.25,P.h*0.25)

        # creating damper on first call and updating it on each call after it
        if self.flag_init == True:
            # Creating object for damper and appending its handle to handle list
            h1, =self.ax.plot(x1,y1, lw=1, c='black')
            v1, =self.ax.plot(x2,y2, lw=1, c='black')
            h2, =self.ax.plot(x3,y3, lw=1, c='black')
            h3, =self.ax.plot(x4,y4, lw=1, c='black')
            v2, =self.ax.plot(x5,y5, lw=1, c='black')
            h4, =self.ax.plot(x6,y6, lw=1, c='black')

            self.handle.append(h1)
            self.handle.append(v1)
            self.handle.append(h2)
            self.handle.append(h3)
            self.handle.append(v2)
            self.handle.append(h4)

        else:
            self.handle[2].set_xdata(x1)
            self.handle[2].set_ydata(y1)
            self.handle[3].set_xdata(x2)
            self.handle[3].set_ydata(y2)
            self.handle[4].set_xdata(x3)
            self.handle[4].set_ydata(y3)
            self.handle[5].set_xdata(x4)
            self.handle[5].set_ydata(y4)
            self.handle[6].set_xdata(x5)
            self.handle[6].set_ydata(y5)
            self.handle[7].set_xdata(x6)
            self.handle[7].set_ydata(y6)
