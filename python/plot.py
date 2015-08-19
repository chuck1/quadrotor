#!/usr/bin/env python

import pylab as pl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import struct

vec3_size = 3 * 4
vec4_size = 4 * 4

float_type_char = 'f'
float_type_size = 4

def read_array(f, c):

        n = struct.unpack('i', f.read(4))[0]
        
        print "read array",n
        
	v = np.zeros((n,c))
	
	for ti in range(n):
		v[ti] = struct.unpack(float_type_char*c, f.read(float_type_size*c))
		if c==1:
			#print v[ti]
			pass

	return v

def read(f, N, c):
	v = np.zeros((N,c))
	
	for ti in range(N):
		v[ti] = struct.unpack(float_type_char*c, f.read(float_type_size*c))
		if c==1:
			#print v[ti]
			pass

	return v

def plots(name, x,Y,xl,yl,L = None,S = None):
	
	if L is None:
		L = ['']
	if S is None:
		S = ['b']
	
	fig = pl.figure(name)
	ax = fig.add_subplot(111)
	
	leg = []
	
	for y,s,l in zip(Y,S,L):
		print np.shape(x),np.shape(y)
		ax.plot(x,y,s)
		leg += [l]
	
	
	ax.set_xlabel(xl)
	ax.set_ylabel(yl)

	ax.legend(leg)

def plotv(name, x,Y,xl,yl,L = None,S = None):
	
	if L is None:
		L = ['']
		S = ['-']
	
	fig = pl.figure(name)
	ax = fig.add_subplot(111)

	leg = []
	
	clr = ['b','g','r','c']
       
                
	for y,s,l in zip(Y,S,L):

                if np.size(y,1) == 3:
                    leg_temp = lambda l: ['x'+l,'y'+l,'z'+l]
                elif np.size(y,1) == 4:
                    leg_temp = lambda l: ['w'+l,'x'+l,'y'+l,'z'+l]

		for i in range(np.size(y,1)):
		    ax.plot(x,y[:,i], clr[i] + s)
		leg += leg_temp(l)
	
	
	ax.set_xlabel(xl)
	ax.set_ylabel(yl)

	ax.legend(leg)

def plotvn(x,Y,xl,L = None,S = None):
	
	if L is None:
		L = ['']
		S = ['-']
	
	fig = pl.figure()
	ax = fig.add_subplot(111)

	leg = []
	
	clr = ['b','g','r','c']
	
	for y,s,l in zip(Y,S,L):
		y = y / np.max(y)

		for i in range(np.size(y,1)):
			ax.plot(x,y[:,i], clr[i] + s)
		leg += ['x'+l,'y'+l,'z'+l]
	
	
	ax.set_xlabel(xl)

	ax.legend(leg)

def size(f):
	old_file_position = f.tell()
	f.seek(0, os.SEEK_END)
	size = f.tell()
	f.seek(old_file_position, os.SEEK_SET)
	return size

def read_cl_x():
    with open("data/cl_x.txt","rb") as f:
    	
    	x_ref = []
    	for i in range(5):
    		x_ref.append(read_array(f, 3))
    	e_x = []
    	for i in range(5):
    		e_x.append(read_array(f, 3))

        print "data/cl_x.txt", np.shape(e_x)

        return e_x, x_ref

e_x, x_ref = read_cl_x()

def read_cl_v():
    with open("data/cl_v.txt","rb") as f:
	
	v_ref = []
	for i in range(4):
		v_ref.append(read_array(f, 3))
	e_v = []
	for i in range(4):
		e_v.append(read_array(f, 3))

        print "data/cl_v.txt", np.shape(e_v)

        return e_v, v_ref



with open("data/jounce.txt","rb") as f:
    jounce = read_array(f, 3)
    print "data/jounce.txt", np.shape(jounce)

with open("data/thrust.txt","rb") as f:
    thrust = read_array(f, 1)
    print "data/thrust.txt",np.shape(thrust)

with open("data/alpha.txt","rb") as f:
    alpha = read_array(f, 3)
    print "data/alpha.txt", np.shape(alpha)

"""
with open("data/att.txt","rb") as f:
	types = 3*3 + (2)*4
	N = size(f)/(float_type_size * types)
	print N
	
	att_e3		= read(f ,N, 4)
	
	q_ref		= read(f ,N, 4)
	q_ref_d		= read(f ,N, 3)
	q_ref_dd	= read(f ,N, 3)
	
	tau_RB		= read(f ,N, 3)
"""
with open("data/plant.txt","rb") as f:
	
	types = 2*4 + 2*3 + 2*1
	
	N = size(f)/(types * float_type_size)
	
	print N
	
	gamma1		= read(f, N, 4)
	gamma1_act	= read(f, N, 4)
	
	pl_tau_RB	= read(f, N, 3)
	pl_f_RB		= read(f, N, 3)
	
	gamma0		= read(f, N, 1)
	gamma0_act	= read(f, N, 1)
"""	
with open("data/brain.txt","rb") as f:
	types = 1*1
	N = size(f)/(types * float_type_size)
	print N
"""

with open("data/telem.txt","rb") as f:
	types = 6*3 + 1*4
	N = size(f)/(types * float_type_size)
	print N
	
	x = read_array(f, 3)
	v = read_array(f, 3)
	a = read_array(f, 3)
	j = read_array(f, 3)
	s = read_array(f, 3)
	q = read_array(f, 4)
	o = read_array(f, 3)


t = np.arange(N) * 0.01

try:
    e_v, v_ref = read_cl_v()
    plotv('ev1', t, [e_v[1]], 't', 'ev1')
except:
    pass

plotv('ex0',t,[e_x[0]],'t','ex0')
plotv('ex1',t,[e_x[1]],'t','ex1')
plotv('ex2',t,[e_x[2]],'t','ex2')
plotv('ex3',t,[e_x[3]],'t','ex3')


#plotv(t,[e2],'t','e2')
#plotv(t,[e3],'t','e3')
#plotv(t,[e4],'t','e4')

#plotv(t,[x],'t','x')

plotv('x_ref[0]',t,[x_ref[0]],'t','x_ref0')

#plotv(t,[x_ref_d],'t','x_ref_d')

plotv('accel',t,[a],'t','a')
plotv('snap',t,[jounce,s], 't', 'jounce',['des','act'],['-','--'])
plotv('rot accel',t,[alpha], 't', 'alpha')
plots('thrust',t,[thrust],'t','thrust')

plotv('q',t,[q],'t','q')

plotv('o',t,[o],'t','o')

"""




#plotvn(t,[e1,e3,a,i],'t',['e1','e3','a','i'],['-','--',':','-.'])
"""

"""

#plots(t,[e1_mag_d],'t','e1_mag_d')
#plots(t,[e1_mag_dd],'t','e1_mag_dd')
plotv(t,[x_ref_d],'t','x_ref_d')
plotv(t,[x_ref_dd],'t','x_ref_dd')
#plotv(t,[f_R],'t','f_R')

#plotv(t,[e3],'t','e3')
plotv(t,[q,q_ref],	't','q',	['','_ref'],['-','--'])
plotv(t,[o,q_ref_d],	't','q_ref_d',	['','_ref'],['-','--'])
plotv(t,[q_ref_dd],'t','q_ref_dd')
#plotv(t,[tau_RB],'t','tau_RB')
"""

#plotv(t,[q[:,1:4]],'t','q')

#plotv(t,[gamma1,gamma1_act],'t','gamma',	['','_act'],['-','--'])

#plotv(t,[pl_tau_RB],'t','plant tau_RB')
#plotv(t,[pl_f_RB],'t','plant f_RB')

def plotpath():
    # calculate axis
    r = np.max(x,0) - np.min(x,0)
    R = max(r) / 2.0
    c = (np.max(x,0) + np.min(x,0)) / 2.0


    fig = pl.figure("path")
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x[:,0],x[:,1],x[:,2],'o')
    ax.plot(x_ref[0][:,0],x_ref[0][:,1],x_ref[0][:,2],'o')

    ax.set_xlim3d([c[0]-R,c[0]+R])
    ax.set_ylim3d([c[1]-R,c[1]+R])
    ax.set_zlim3d([c[2]-R,c[2]+R])

plotpath()

pl.show()





