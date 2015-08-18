import numpy as np
import vec
import math

e2 = np.array([0.0,0.0,1.0])

class Quat:
	def __init__(self, q = None, s = None, v = None, theta = None, v1 = None, v2 = None):
		
		if not q is None:
			# copy
			self.s = q.s
			self.v = q.v
		if not s is None:
			# components
			self.s = s
			self.v = np.array(v)
		elif (not v1 is None) and (not v2 is None):
			# rotation between two vectors
			v1 = vec.normalize(v1)
			v2 = vec.normalize(v2)
			self.v = np.cross(v1,v2)
			self.s = np.dot(v1,v2)
			self = self.normalize()
		elif not theta is None:
			# axis-angle
			self.s = math.cos(theta/2.0)
			self.v = np.array(v) * math.sin(theta/2.0)
		else:
			# zero
			self.s = 1.0
			self.v = np.zeros(3)

		if self.isnan():
			raise ValueError('Quat nan')
		
	def normalize(self):
		m = self.mag()
		s = self.s / m
		v = self.v / m
		return Quat(s=s,v=v)

	def mag(self):
		return math.sqrt(np.sum(np.square(self.v)) + self.s**2)

	def __mul__(self, q2):
		s = self.s * q2.s - np.dot(self.v, q2.v)
		v = self.s * q2.v + q2.s * self.v + np.cross(self.v, q2.v)
		return Quat(s=s,v=v)

	def conj(self):
		return Quat(s=self.s, v=-self.v)

	def rotate(self, v):
		qv = Quat(s=0, v=v)
		
		res = self * qv * self.conj()
		
		return res.v
	def isnan(self):
		if math.isnan(self.s):
			return True
		if any(np.isnan(self.v)):
			return True
		return False
	def to_omega(self, dt):
		m = vec.mag(self.v)
		
		#print self.s,s

		if m > 0.0:
			omega = 2.0 * math.asin(m) / dt * vec.normalize(self.v)
		else:
			omega = np.zeros(3)
			
		ver = False
		if ver:
			print 'dt   ',dt
			print 'm    ',m
			print 'r    ',self.s,self.v
			print 'omega',omega



		return omega
		
if __name__ == '__main__':
	r = Quat(theta = math.pi/10.0, v = [0,1,0])
	
	#v = np.array([1.0,0.0,0.0])
	
	#print v
	#print r.rotate(v)
	#print r.conj().rotate(v)
	
	q1 = Quat()
	q2 = r * q1

	print 'q1',q1.v
	print 'q2',q2.v
		
	print 'om',r.to_omega(0.01)
	

