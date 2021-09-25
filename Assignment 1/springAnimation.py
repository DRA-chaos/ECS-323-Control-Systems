

##Rita Abani 19244
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import springParam as P


class springAnimation:
    '''
        Create spring animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        self.length=P.length
        self.width=P.width
        plt.axis([-2.0*P.length, 2.0*P.length, -2.0*P.length, 2.0*P.length]) # Change the x,y axis limits
        plt.plot([0, P.length], [0, 0],'k--')    # Draw a base line

      
    def drawspring(self, u):
        
       
        z = u[0]

       
        X = [z, z + P.length]
        Y = [0, 0]

       
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, = self.ax.plot(X, Y, lw=50, c='blue')
            self.handle.append(line)
            self.flagInit=False
        else:
            self.handle[0].set_xdata(X)   
            self.handle[0].set_ydata(Y)
