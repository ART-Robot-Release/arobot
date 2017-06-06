# The example of interpolation

import numpy as np
import pylab as pl
from scipy import interpolate


x = np.linspace(0, 2*np.pi+np.pi/4, 10)
y = np.sin(x)

x_new = np.linspace(0, 2*np.pi+np.pi/4,100)
f_linear = interpolate.interp1d(x, y)
tck = interpolate.splrep(x, y)
y_bspline = interpolate.splev(x_new, tck)

pl.plot(x, y, "o", label=u"The raw data")
pl.plot(x_new, f_linear(x_new), label = u"linear interpolate")
pl.plot(x_new, y_bspline, label = u"B-spline interpolate")
pl.legend()
pl.show()

