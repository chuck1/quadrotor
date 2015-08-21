#!/usr/bin/env python

import math
import cmath
import operator
import numpy as np
import pylab as pl

def transfer(x, Z, P):
    """
    x - input
    z - zeros
    p - poles
    """
    
    rz = reduce(operator.mul, list(x - z for z in Z))
    rp = reduce(operator.mul, list(x - p for p in P))
    
    h = rz/rp
    
    #print rz
    #print rp

    #print h

    return abs(h), cmath.phase(h)

def transfer_sine(wh0, Z, P):
    x = complex(wh0,wh0)
    
    return transfer(x, Z, P)

Z = [-1, -1, -2, -2, 0] 
P = [-1, -1]

print "numerator   poly",np.poly(Z)
print "denominator poly",np.poly(P)

wh0 = np.arange(-math.pi/2, math.pi/2, 0.01)

f = np.vectorize(lambda x: transfer_sine(x, Z, P))

a,p = f(wh0)

#print a
#print p

if 0:
    pl.figure()
    pl.plot(wh0, a)
    pl.xlabel('wh0')
    pl.ylabel('amplitude')
    pl.figure()
    pl.plot(wh0, p)
    pl.xlabel('wh0')
    pl.ylabel('phase')






pl.show()

