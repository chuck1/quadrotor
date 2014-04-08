import pylab as pl
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import itertools
import math


import Quadcopter
import control
import quaternion as qt
from visual import *
import vec

class Sim:
	def __init__(self, dt, N):
		self.dt = dt
		self.N = N

		self.t = np.arange(N) * dt

		self.c = Quadcopter.Quad(self.t)
		self.b = control.Brain(self.c)

	def run(self):
		for ti in range(1,self.N):
			#if (ti % (self.N / 100)) == 0:
			#	print ti
			try:
				self.b.step(ti-1)
			except control.ErrorListEmpty:
				break
			
			self.c.step(ti)
		
		#c.write(2)



def sub1(p, param, ts, N):

	a, b, c, d = p

	if ts[a,b,c,d] != 0.0:
		return ts, N
	
	s = Sim(0.01,N)
			
	s.b.ctrl_position.C5[0,0] = param[0,a]
	s.b.ctrl_position.C5[1,1] = param[0,a]
	
	s.b.ctrl_position.C6[0,0] = param[1,b]
	s.b.ctrl_position.C6[1,1] = param[1,b]
					
	
	s.b.ctrl_attitude.C1[0,0] = param[2,c]
	s.b.ctrl_attitude.C1[1,1] = param[2,c]
	s.b.ctrl_attitude.C1[2,2] = param[2,c]
	
	s.b.ctrl_attitude.C2[0,0] = param[3,d]
	s.b.ctrl_attitude.C2[1,1] = param[3,d]
	s.b.ctrl_attitude.C2[2,2] = param[3,d]
	
	s.b.objs = [control.Move([1.0,0.0,0.0],[0.01,0.01,0.01])]
					
	s.run()
	
	ti = s.b.obj.ti_1
	if ti > -1:
		if ti < N:
			N = ti
			index = [a,b,c,d]
				
	ts[a,b,c,d] = s.b.obj.ts
				
	ind[a,b,c,d] = [a,b,c,d]
					
	print "{0:0.2e} {1:0.2e} {2:0.2e} {3:0.2e} {4}".format(
			param[0,a],
			param[1,b],
			param[2,c],
			param[3,d],
			s.b.obj.ts)


	return ts, N





# parameter space
NP = 4 # num params
NG = 13 # num grid points

# force odd
NG += (NG+1) % 2

center = np.array([20.5, 20.5, 40.5, 40.5])
length = np.array([20.0, 20.0, 40.0, 40.0])

index = None

for it in range(10):
	param = np.zeros((NP,NG))
	
	for a in range(NP):
		for b in range(NG):
			param[a,b] = center[a] + length[a] * (float(b) * 2.0 / (NG - 1.0)  - 1.0)
	
	print param

	#ts = np.zeros((2,2,2,2))
	ts = np.zeros([NG]*NP)
	ind = np.zeros([NG]*NP + [NP])

	print np.shape(ts)
	print np.shape(ind)
	
	N = 2000
	print range(NG)
	prod = list(itertools.product(range(NG), repeat = NP))



	print 'center',center
	print 'param'
	print param

	p = [(NG-1)/2] * NP

	print 'center index',p

	ts, N = sub1(p, param, ts, N)
	
	for p in prod:
		ts, N = sub1(p, param, ts, N)
	
	ts_min = np.min(ts[(ts > -1.0)])
	
	index = ind[ts == ts_min][0]
	
	for a in range(4):
		center[a] = param[a,index[a]]
	
	
	print 'ts',ts_min
	
	length *= 0.5
	
	
		
	

















#b.ctrl_position.fill_xref([1.0, 1.0, 0.0])

"""
b.objs = [
		control.Move([1.0,0.0,0.0],[0.01,0.01,0.01]),
		control.Move([1.0,1.0,0.0],[0.01,0.01,0.01]),
		control.Move([0.0,1.0,0.0],[0.01,0.01,0.01]),
		control.Move([0.0,0.0,0.0],[0.01,0.01,0.01]),
		control.Move([0.0,0.0,1.0],[0.01,0.01,0.01]),
		control.Move([1.0,0.0,1.0],[0.01,0.01,0.01]),
		control.Move([1.0,1.0,1.0],[0.01,0.01,0.01]),
		control.Move([0.0,1.0,1.0],[0.01,0.01,0.01]),
		control.Move([0.0,0.0,1.0],[0.01,0.01,0.01],mode = 1)]
"""

"""
b.objs = [
		control.Orient(qt.Quat(theta = math.pi, v = [1.0, 0.0, 0.0]), mode = control.ObjMode.hold)]
"""

"""
b.objs = [
		control.Path(lambda t: np.array([math.sin(t * 4.0 / math.pi),t,0.0]))]
"""

"""
should_plot = False
if should_plot:
	c.plot()
	b.ctrl_position.plot()
	b.ctrl_attitude.plot()
	b.plot()
	#c.plot3()
	pl.show()
"""


