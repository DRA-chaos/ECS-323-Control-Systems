

##Rita Abani 19244


import numpy as np
import springParam as P 
import matplotlib.pyplot as plt 
from matplotlib.widgets import damper

class dampers:
    ''' This class inherits the damper class. Its purpose is to 
        help organize one or more dampers in an axes.'''

  
    def __init__(self):
        # Creates a figure and axes for the dampers.
        self.fig,self.ax = plt.subplots() 
        plt.axis("off")               # Suppress the axis from showing.

        self.num_of_dampers = 2 
        
        self.Sz = mydamper(self.num_of_dampers,1,
            2*P.L,-2*P.L,P.z0,1,'z')
        self.Stheta = mydamper(num_of_dampers = self.num_of_dampers,
            damper_number = 2, maxV = 90,minV = -90,intV =0, 
            gain = np.pi/180, name ='theta')

        
        plt.draw()


    def getInputValues(self):
        
        ################################################################              
        values = [self.Sz.getValue(),self.Stheta.getValue()]
        ################################################################
        return values



class mydamper:

    def __init__(self,num_of_dampers=1, damper_number=1, maxV = 1, 
        minV = -1, intV = 0, gain = 1, name = 'damper'):
       
        self.data = 0            
        self.name = name         
        self.maxValue = maxV     
        self.minValue = minV     
        self.initValue = intV    
        self.gain = gain         
        self.damper_length = 0.5 
        self.damper_width = 0.03 

        
        # Sets the position and color of the damper

        # The horizontal position of the damper in the figure.
        hpos = 0.5 - 0.6/(num_of_dampers+1)*damper_number 

        # Specify a subsection of the current axes where the damper will go.
        self.axdamper = plt.axes([0.5-self.damper_length/2,hpos, self.damper_length, 
            self.damper_width], axisbg = 'orange')

        # Instantiate a damper, and create a handle to it
        self.damperHandle = damper(self.axdamper, self.name, self.minValue,
            self.maxValue, valinit = self.initValue)

        # When a change occurs on the damper, the function self.update
        # will be called.
        self.damperHandle.on_changed(self.update)

        

    # Updates the value of the damper when the damper is changed
    def update(self,val):
        self.data = self.damperHandle.val

    # Returns the current value of the damper(s)
    def getValue(self):
        return self.data*self.gain















    

   
