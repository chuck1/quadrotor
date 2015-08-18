import numpy as np
import pylab as pl

import control
import vec

class Position1:
	def __init__(self, c):
		self.c = c
		
		C5_11 = 3.1
		C5_22 = 3.1
		C5_33 = 0.15

		C6_11 =  3.0
		C6_22 =  3.0
		C6_33 = 10.0

		L5_11 = 0.0
		L5_22 = 0.0
		L5_33 = 0.0
		

		self.C5 = np.array([
				[C5_11,0,0],
				[0,C5_22,0],
				[0,0,C5_33]])
		self.C6 = np.array([
				[C6_11,0,0],
				[0,C6_22,0],
				[0,0,C6_33]])
		self.L5 = np.array([
				[L5_11,0,0],
				[0,L5_22,0],
				[0,0,L5_33]])
		
		self.e5 = np.zeros((c.N, 3))
		self.e6 = np.zeros((c.N, 3))
		self.chi5 = np.zeros((c.N, 3))


		
		self.e5_mag_d = np.zeros(c.N)




		self.x_ref = np.zeros((c.N, 3))
		self.x_refd = np.zeros((c.N, 3))
		self.x_refdd = np.zeros((c.N, 3))

		self.v_ref = np.zeros((c.N, 3))

		self.f_R = np.zeros((c.N, 3))

		self.flag_converged = False
		
	def fill_xref(self, ti1, x):
		for ti in range(ti1, self.c.N):
			self.x_ref[ti] = np.array(x)

	def fill_xref_parametric(self, ti1, f):
		for ti in range(ti1, self.c.N):
			t = self.c.t[ti]
			self.x_ref[ti] = np.array(f(t))
		
		#pl.plot(self.c.t, self.x_ref)
		
	def get_f6(self, e5, e6):
		return -np.dot(self.C6,e6) - e5
	def step(self, ti, ti_0):
		dt = self.c.t[ti] - self.c.t[ti-1]
		
		# reference
		self.x_refd[ti] = (self.x_ref[ti] - self.x_ref[ti-1]) / dt
		
		if ti_0 > 1:
			self.x_refdd[ti] = (self.x_refd[ti] - self.x_refd[ti-1]) / dt
		
		# step position error
		self.e5[ti] = self.x_ref[ti] - self.c.x[ti]
		
		if ti_0 > 0:
			self.chi5[ti] = self.chi5[ti-1] + self.e5[ti] * dt
		
		if ti_0 > 0:
			self.e5_mag_d[ti] = (vec.mag(self.e5[ti]) - vec.mag(self.e5[ti-1])) / dt
		
		# step v_ref before stepping e6
		self.v_ref[ti] = np.dot(self.C5, self.e5[ti]) + self.x_refd[ti] + np.dot(self.L5, self.chi5[ti])
		
		# step velocity error
		self.e6[ti] = self.v_ref[ti] - self.c.v[ti]
		
		e5_mag = vec.mag(self.e5[ti])
		
		if self.obj:
			close = all(np.absolute(self.e5[ti]) < self.obj.thresh)
			slow = all(np.absolute(self.e6[ti]) < self.obj.thresh)
			if ti_0 > 1:
				"""
				if self.e5_mag_d[ti] < 0.0:
					if self.e5_mag_d[ti-1] > 0.0:
						# local maximum error
						if e5_mag < self.e5_local_max:
							# converging
							if close:
								# converged
								self.obj.flag_complete = True
						
						self.e5_local_max = e5_mag
				"""
				if close and slow:
					if self.obj.flag_settled == False:
						if self.e5_mag_d[ti] > -0.01:
							if self.e5_mag_d[ti] < 0.0:
								# settled
								self.obj.settle(ti)
								if self.obj.mode == 0: #control.ObjMode.normal:
									self.obj.flag_complete = True


	def set_obj(self, ti, obj):
		self.obj = obj
		
		# reset
		self.e5_local_max = 0.0
		self.flag_converged = False
		
		if isinstance(obj, control.Move):
			self.fill_xref(ti, self.obj.x2)
		elif isinstance(obj, control.Path):
			self.fill_xref_parametric(ti, self.obj.f) 
		
		self.obj.start(self.c, ti)
		
		
	def get_force_rotor(self, ti, ti_0):
		e5 = self.e5[ti]
		e6 = self.e6[ti]
		chi5 = self.chi5[ti]
		
		f6 = self.get_f6(e5,e6)

		C5 = self.C5
		L5 = self.L5
		
		m = self.c.m
		g = self.c.gravity		

		f_D = self.c.get_force_drag(ti)

		x_refdd = self.x_refdd[ti]
		
		temp1 = np.dot(C5, e6 - np.dot(C5, e5) - np.dot(L5, chi5))
		
		temp2 = np.dot(L5, e5)
		
		f_R = m * (-f6 + temp1 + x_refdd + temp2 - g) - f_D
		
		self.f_R[ti] = f_R
		
			
		ver = True
		ver = False
		if ver:
			print 'f6   ' ,f6
			print 'temp1' ,temp1
			print 'xrefdd',xrefdd
			print 'temp2 ',temp2
			print 'g     ',g
			print 'f_D   ',f_D
			print 'f_R   ',f_R

		return f_R
	def plot(self):
		t = self.c.t
		
		# f
		fig = pl.figure()
		
		ax = fig.add_subplot(111)
		ax.set_ylabel('f_R')
		
		ax.plot(t, self.f_R)

		# x
		fig = pl.figure()
		
		x = self.c.x[:,0]
		y = self.c.x[:,1]
		z = self.c.x[:,2]
		xr = self.x_ref[:,0]
		yr = self.x_ref[:,1]
		zr = self.x_ref[:,2]
		
		ax = fig.add_subplot(111)
		ax.set_xlabel('t')
		ax.set_ylabel('x')
		
		ax.plot(t,x,'b-')
		ax.plot(t,y,'g-')
		ax.plot(t,z,'r-')
		ax.plot(t,xr,'b--')
		ax.plot(t,yr,'g--')
		ax.plot(t,zr,'r--')
		
		ax.legend(['x','y','z','xr','yr','zr'])
		
		# v
		fig = pl.figure()
		
		x = self.c.v[:,0]
		y = self.c.v[:,1]
		z = self.c.v[:,2]
		xr = self.v_ref[:,0]
		yr = self.v_ref[:,1]
		zr = self.v_ref[:,2]
		
		ax = fig.add_subplot(111)
		ax.set_xlabel('t')
		ax.set_ylabel('v')
		
		ax.plot(t,x,'b-')
		ax.plot(t,y,'g-')
		ax.plot(t,z,'r-')
		ax.plot(t,xr,'b--')
		ax.plot(t,yr,'g--')
		ax.plot(t,zr,'r--')
		
		ax.legend(['x','y','z','xr','yr','zr'])
		
		# e	
		fig = pl.figure()
		
		ax = fig.add_subplot(221)
		ax.set_ylabel('e_5')
		ax.plot(t, self.e5)
	
		ax = fig.add_subplot(222)
		ax.set_ylabel('e_6')
		ax.plot(t, self.e6)
		
		# e5_mag_d
		fig = pl.figure()
		
		ax = fig.add_subplot(111)
		ax.set_ylabel('e5_mag_d')
		ax.plot(t, self.e5_mag_d)
		
	
	
