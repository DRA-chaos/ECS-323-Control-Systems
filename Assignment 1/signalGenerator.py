

##Rita Abani 19244
import numpy as np


class signalGenerator:
    
    
    def __init__(self, amplitude=1, frequency=1, y_offset=0):
        ''' 
            amplitude - signal amplitude.  Standard deviation for random.
            frequency - signal frequency
            y_offset  - signal y-offset
        '''
        self.amplitude = amplitude
        self.frequency = frequency
        self.y_offset = y_offset

    def square(self, t):
        if t % (1.0/self.frequency) <= 0.5/self.frequency:
            out = self.amplitude + self.y_offset
        else:
            out = - self.amplitude + self.y_offset
        return [out]
        

    def sin(self, t):
        out = self.amplitude*np.sin(2*np.pi*self.frequency*t) + self.y_offset
        return [out]
        #  returns a list of length 1
