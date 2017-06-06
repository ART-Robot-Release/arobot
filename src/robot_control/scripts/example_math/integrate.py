# The example of interpolation

import numpy as np
import matplotlib as plt
from scipy import integrate

def half_circle(x):
    return (1-x**2)**0.5

half_area, err = integrate.quad(half_circle, -1, 1)

print half_area * 2

def half_sphere(x, y):
    return (1-x**2-y**2)**0.5

out = integrate.dblquad(half_sphere, -1, 1, lambda x:-half_circle(x), lambda x:half_circle(x))

print out
