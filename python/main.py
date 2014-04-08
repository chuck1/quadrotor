import pylab as pl
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

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

		self.t = np.arange(self.N) * dt

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
		
		self.c.write(2)

	def plot(self):
		self.c.plot()
		self.b.ctrl_position.plot()
		self.b.ctrl_attitude.plot()
		self.b.plot()
		#self.c.plot3()
		pl.show()


			
s = Sim(0.01,400)

C5 = 4.7#6.75# 2.09
C6 = 5.5#7.06#12.60

C1 = 79.0#11.59#18.07
C2 = 91.0#15.81#10.80

s.b.ctrl_position.C5[0,0] = C5
s.b.ctrl_position.C5[1,1] = C5
	
s.b.ctrl_position.C6[0,0] = C6
s.b.ctrl_position.C6[1,1] = C6

	
s.b.ctrl_attitude.C1[0,0] = C1
s.b.ctrl_attitude.C1[1,1] = C1
s.b.ctrl_attitude.C1[2,2] = C1
	
s.b.ctrl_attitude.C2[0,0] = C2
s.b.ctrl_attitude.C2[1,1] = C2
s.b.ctrl_attitude.C2[2,2] = C2
	
s.b.objs = [control.Move([1.0,0.0,0.0],[0.01,0.01,0.01], mode = 0)]

s.run()

s.plot()




