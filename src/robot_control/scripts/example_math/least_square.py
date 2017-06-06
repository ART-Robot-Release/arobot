# The example of the least square method

import numpy as np
from scipy.optimize import leastsq
import pylab as pl

def func(x, p):
    """
    The function of the fitting  A*sin(2*pi*k*x + th)
    """
    
    A, k, th = p
    return A*np.sin(2*np.pi*k*x + th)

def residuals(p, y, x):
    """
    The x,y and the cofficients - p
    """
    return y - func(x, p)

x = np.linspace(0, -2*np.pi, 100)
A, k, th = 10, 0.34, np.pi/6  # The real parameters

y0 = func(x, [A, k, th]) # The real data 
y1 = y0 + 2 * np.random.randn(len(x)) # Add the noise

p0 = [7, 0.2, 0] # The first data we assume 

# Call the leastsq
# The residuals is the function for calculating the error
# p0 is the initial value 
# args the the input data 
plsq = leastsq(residuals, p0, args=(y1, x))

print u"The real data:", [A, k, th]
print u"The arg of fitting", plsq[0] 

pl.plot(x, y0, label = u"Real Data")
pl.plot(x, y1, label = u"The data with noise.")
pl.plot(x, func(x, plsq[0]), label = u"Fitting data")
pl.legend()
pl.show()
