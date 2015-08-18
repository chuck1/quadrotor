import numpy as np
import pylab as pl

class PID:
	def __init__(self,p,i,d):
		self.i = i
		self.p = p
		self.d = d
		self.x = []
		self.y = []
		self.integral = 0.
		self.e = []
		self.v = []
		self.f = []
	def step(self, x, dt):
		self.x.append(x)
		self.y.append(self.target)
		
		self.e.append(self.target - x)
		
		if len(self.e) > 1:
			self.v.append( (self.e[-2] - self.e[-1]) / dt )
		else:
			self.v.append(0)
		
		self.integral += self.e[-1] * dt
		
		f = (self.i * self.integral) + (self.p * self.e[-1]) - (self.d * self.v[-1])
		self.f.append(f)

		#print 'e v f'
		#print self.e[-1], self.v[-1], self.f[-1]

	def plot(self):
		fig = pl.figure()
		ax = fig.add_subplot(221)
		ax.plot(self.x)
		ax.plot(self.y)
		ax.set_ylabel('position')

		ax = fig.add_subplot(222)
		ax.plot(self.v)
		ax.set_ylabel('velocity')

		ax = fig.add_subplot(223)
		ax.plot(self.f)




