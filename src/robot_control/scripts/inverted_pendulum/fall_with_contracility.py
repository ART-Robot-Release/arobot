"""

Inverted pendulum :

The relationship between the fall movement and contractility.

"""

from math import sin, cos
import numpy as np
import matplotlib as plt
from scipy.integrate import odeint

g = 9.8
height = 0

class InvertedPendulum(object):
    def __init__(self, tao, M, f_model):
        self.tao, self.M, self.f_model = tao, M, f_model
        self.init_status = np.array([0.0, 0.0, 0.0, 0.0])

    def equations(self, w, t):
        """
        The equation.
        """
        tao, M, f_model = self.tao, self.M, self.f_model
    
        

        r, th, v1, v2 = w;
        dr = v1;
        dth = v2;

        if (f_model == 0):
            f = 0.0
        elif (f_model == 1):
            f =  M*g*cos(th) - M*r*dth*dth
        elif (f_model == 2):
            f = M*g
        elif (f_model == 3): 
            f = M*g/cos(th)

        # eq of v1
        # r^2 * dv2 + 2r*dr*dth - g*r*sin(th) - tao/M = 0
        a = 0
        b = r*r
        c = 2.0*r*dr*dth - g*r*sin(th) - tao/M 

        # eq of v2
        # dv1 - r*dth^2 +g*cos(th) - f/M = 0
        d = 1
        e = 0
        f1 = -r*dth*dth + g*cos(th) - f/M
        dv1, dv2 = np.linalg.solve([[a,b], [d,e]], [-c, -f1])
        return np.array([dr, dth, dv1, dv2])

def inverted_pendulum_odeint(pendulum, ts, te, tstep):
    """
    solve the differential equation. retrun X-Y
    """

    t = np.arange(ts, te, tstep)
    track = odeint(pendulum.equations, pendulum.init_status, t, mxstep=5000)
    r_array, th_array = track[:, 0], track[:, 1]
    x = r_array * np.sin(th_array)
    y = r_array * np.cos(th_array)
    pendulum.init_status = track[-1, :].copy() #Set the last status
    return [x, y]

def plotThePic(x, y, plts, num):
    plts.plot(x, y , 'o')
   # plts.axis([0, 1000, -100, 600])
    for i in range(0, len(x)):
        plts.plot([0, x[i]], [0, y[i] + height])


if __name__ == "__main__":
    import pylab as pl

    for i in range(0,4):
        print(i)
        pendulum = InvertedPendulum(0, 1, i)
        pendulum.init_status[:2] = 5.0, 0.5 # r, th
        x, y = inverted_pendulum_odeint(pendulum, 0, 2, 0.02)
        plotThePic(x, y, pl.subplot(2,2,i+1), i)
        pl.title(" The plot of %d" % (i))
    pl.show()
