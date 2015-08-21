#!/usr/bin/env python

import math
import cmath
import operator
import numpy as np
import pylab as pl
import scipy.signal
import sympy

def residue_expr(s,r,p,k):
    N = len(r)
    
    expr = 0

    for i in range(N):
        n = N + 1 - i
        for j in range(i, N):
            
            k = j - i
            #k = j + (1 - i)

            print "N={} n={} i={} j={} k={} r[j]={:5}".format(N, n, i, j, k, r[j])
            expr += r[j] / ((s - p[i])**k)

    return expr

def residue_denominator(P):
    cP = np.poly(P)
    r,p,k = scipy.signal.residue([1],cP)
    if k[0] != 0:
        raise ValueError()
    return r,p,k
    
def zp_expr(s,Z,P):
    pZ = np.poly(Z)
    pP = np.poly(P)

    a = np.polyval(pZ, s)
    b = np.polyval(pP, s)

    print "zp_expr"

    print "  a",a
    print "  b",b

    return a / b
   

Z = np.array([-1., -1., 0.])
#Z = np.array([-1., -1.])

P = np.array([-2, -2, -1., -1., 0.])
P = np.array([-1., -1., 0.])


pZ = np.poly(Z)
pP = np.poly(P)

print " Z",Z
print " P",P
print "pZ",pZ
print "pP",pP

# calculate 

#print "r",r
#print "p",p
#print "k",k

s = sympy.symbols('s')

expr0 = zp_expr(s,Z,P)
#expr0 = zp_expr(s,Z,[0])


r,p,k = residue_denominator(P)

expr1 = residue_expr(s,r,p,k) * np.polyval(pZ,s)
#expr1 = residue_expr(s,r,p,k)


#sympy.pprint(expr)
#print

print "original"
print
sympy.pprint(sympy.simplify(expr0))
print

print "from residue"
print
sympy.pprint(sympy.factor(expr1))
print
sympy.pprint(sympy.expand(expr1))
print
sympy.pprint(sympy.cancel(expr1))
print
sympy.pprint(sympy.simplify(expr1))
print
 


