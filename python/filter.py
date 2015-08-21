#!/usr/bin/env python

#import sympy
import os
import math
import cmath
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(z, c_x, c_y):
    X = sum(c * (z**(-k)) for c,k in zip(c_x, range(len(c_x))))
    Y = sum(c * (z**(-k)) for c,k in zip(c_y, range(len(c_y))))
    
    H = X/Y

    #print H

    #print abs(H)

    #print cmath.phase(H)

    return abs(H), cmath.phase(H)

def analyze_sine(w, c_x, c_y):
    z = complex(math.cos(w), math.sin(w))
    return analyze(z, c_x, c_y)

a = 1
b = 2
c = 2
d = 2

w0 = math.pi
w = np.arange(-w0,w0,0.010)

print w

#c_y = [a+b+c+d, -b, -c, -d]
c_y = [a+b, -b]
c_x = [a]
#c_y = [2, -1]
#c_x = [1]

f = np.vectorize(lambda w: analyze_sine(w, c_x, c_y))

a,p = f(w)

print a
print p

root = '/home/chuck/home/var/www/html/md_files/projects/arduino_drone/math/math/plot/filter'

plt.ioff()

fig = plt.figure()
#plt.loglog(w,a)
plt.plot(w,a)
plt.savefig(os.path.join(root,'a.png'))


