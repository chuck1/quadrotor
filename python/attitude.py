
import math
import pylab as pl
import numpy as np

import control
import vec

class Attitude1:
	def __init__(self, c):
		self.c = c
	
		# parameters
		C1_11 = 5.2
		C1_22 = 5.2
		C1_33 = 5.2

		C2_11 = 24.0
		C2_22 = 24.0
		C2_33 = 24.0

		L1_11 = 0.0
		L1_22 = 0.0
		L1_33 = 0.0

		self.C1 = np.array([
				[C1_11,0,0],
				[0,C1_22,0],
				[0,0,C1_33]])
		
		self.C2 = np.array([
				[C2_11,0,0],
				[0,C2_22,0],
				[0,0,C2_33]])
		
		self.L1 = np.array([
				[L1_11,0,0],
				[0,L1_22,0],
				[0,0,L1_33]])
		
		# variables
		self.e1 = np.zeros((c.N,3))
		self.chi1 = np.zeros((c.N,3))

		self.e2 = np.zeros((c.N, 3))
			
		self.theta_ref = np.zeros((c.N,3))
		self.theta_refd = np.zeros((c.N,3))
		self.theta_refdd = np.zeros((c.N,3))

		self.omega_ref = np.zeros((c.N,3))

		self.tau_RB = np.zeros((c.N,3))
	def set_q_reference(self, ti, q):
		self.theta[ti] = vec_to_euler(q.v / mag(q.v))
		
	def get_f2(self, e1, e2):
		return -np.dot(self.C2,e2) - e1
		
	def step(self, ti):
		# refernce values must be set before stpping
		dt = self.c.t[ti] - self.c.t[ti-1]
		
		# reference
		if ti > 0:
			self.theta_refd[ti] = (self.theta_refd[ti] - self.theta_refd[ti-1]) / dt
			
		if ti > 1:
			self.theta_refdd[ti] = (self.theta_refd[ti] - self.theta_refd[ti-1]) / dt
		
		# tracking error
		self.e1[ti] = self.theta_ref[ti] - self.c.theta[ti]
		self.chi1[ti] = self.chi1[ti-1] + self.e1[ti] * dt
		
		# step omega_ref before stepping e2
		self.omega_ref[ti] = np.dot(self.C1, self.e1[ti]) + self.theta_refd[ti] + np.dot(self.L1, self.chi1[ti])
		
		# step e2
		self.e2[ti] = self.omega_ref[ti] - self.c.omega[ti]
		
		
		
		ver = False
		if ver:
			print 'theta_ref',self.theta_ref[ti]
			print 'theta    ',self.c.theta[ti]
			print 'e1       ',self.e1[ti]
		
	def get_tau_rotor_body(self, ti):
		# require error values
		self.step(ti)
		
		
		e1 = self.e1[ti]
		e2 = self.e2[ti]
		
		f2 = self.get_f2(e1,e2)
		
		chi1 = self.chi1[ti]
		
		C1 = self.C1
		L1 = self.L1
		theta_refdd = self.theta_refdd[ti]
		
		A5  = self.c.get_A5(ti)
		A5d = self.c.get_A5d(ti)
		A5inv = self.c.get_A5inv(ti)
		
		omega = self.c.omega[ti]
		
		temp = np.dot(np.dot(C1, A5), e2 - np.dot(C1,e1) - np.dot(L1,chi1))
		
		temp2 = -f2 + temp + theta_refdd + np.dot(L1,e1) - np.dot(A5d,omega)
			
		temp3 = np.cross(omega, np.dot(self.c.I, omega))
	
		temp4 = np.dot(np.dot(self.c.I, A5inv), temp2)
	
		tau_RB = temp4 + temp3

		ver = True
		ver = False
		if ver:		
			print 'A5    ',A5
			print 'A5inv ',A5inv
			print 'A5d   ',A5d

			print 'theta_refdd',theta_refdd
		
			print 'temp  ',temp
			print 'temp2 ',temp2
			print 'temp3 ',temp3
			print 'temp4 ',temp4
			print 'e1    ',e1
			print 'e2    ',e2
			print 'A5    ',A5
			print 'C1    ',C1
			print 'f2    ',f2
			print 'temp  ',temp
			print 'tau_RB',tau_RB
	
		if any(np.isnan(tau_RB)):
			raise ValueError('nan')
		
		self.tau_RB[ti] = tau_RB
		
		return tau_RB


class Attitude2:
	def __init__(self, c):
		self.c = c
		
		self.q_ref = np.empty(c.N, dtype=object)
	
		self.K_PH = 1.0
		self.K_Pz = 1.0
		
		self.K_D = np.array([
				1.0,
				1.0,
				1.0])

		self.tau_RB = np.zeros((c.N,3))

	def set_q_reference(self, ti, q):
		self.q_ref[ti] = q
		
	def get_tau_RB(self, ti):
		
		q = self.q_ref[ti]

		alpha_H = math.acos(1.0 - 2.0*(q.v[0]**2 + q.v[1]**2))
		psi = 2.0 * math.atan2(q.v[2],q.s)
		
		rx = (math.cos(psi/2.0) * q.v[0] - math.sin(psi/2.0) * q.v[1]) / math.sin(alpha_H/2.0)
		ry = (math.sin(psi/2.0) * q.v[0] + math.cos(psi/2.0) * q.v[1]) / math.sin(alpha_H/2.0)


		beta_H = math.atan2(ry,rx)
		gamma_H = math.atan2(rx,-ry)
		
		tau_PH = self.K_PH * alpha_H
		tau_Pz = self.K_Pz * psi
		
		tau_D = np.multiply(self.K_D, self.c.omega[ti])
		
		tau_RB = np.array([
				tau_PH * math.cos(beta_H),
				tau_PH * math.sin(beta_H),
				tau_Pz])
		
		tau_RB += tau_D
		
		self.tau_RB[ti] = tau_RB
		
		return tau_RB
		
	def plot(self):
		fig = pl.figure()
		
		ax = fig.add_subplot(111)
		ax.plot(self.c.t, self.tau_RB)
		ax.set_xlabel('t')
		ax.set_ylabel('tau_RB')

class Attitude3:
	def __init__(self, c):
		self.c = c
		
		self.q_ref = np.empty(c.N, dtype=object)
	
		C1 = 4.0
	
		self.C1 = np.array([
				[C1,0.0,0.0],
				[0.0,C1,0.0],
				[0.0,0.0,C1]])
		
		C2 = 3.0
		
		self.C2 = np.array([
				[C2,0.0,0.0],
				[0.0,C2,0.0],
				[0.0,0.0,C2]])
		
		self.e1 = np.empty(c.N, dtype=object)
		self.e2 = np.zeros((c.N,3))
		
		self.e1_mag_d = np.zeros(c.N)

		self.q_refd = np.zeros((c.N,3))
		self.q_refdd = np.zeros((c.N,3))
		self.omega_ref = np.zeros((c.N,3))
		
		self.tau_RB = np.zeros((c.N,3))

		self.obj = None

	def set_q_reference(self, ti, q):
		self.q_ref[ti] = q
	def set_obj(self, ti1, obj):
		self.obj = obj
		
		for ti in range(ti1, self.c.N):
			self.q_ref[ti] = obj.q
		
	def step(self, ti, ti_0):
		dt = self.c.t[ti] - self.c.t[ti-1]
		
		q = self.c.q[ti]
		q_ref = self.q_ref[ti]
		
		self.e1[ti] = q_ref * q.conj()
		
		q_ref_0 = self.q_ref[ti-1]
		q_ref_1 = self.q_ref[ti-0]
		
		# q_refd
		if ti_0 > 1:
			r = q_ref_1 * q_ref_0.conj()
			q_refd_1 = r.to_omega(dt)
			#print 'r',r.s,r.v
		else:
			q_refd_1 = np.zeros(3)		
		
		# clamp
		q_refd_1 = vec.clamparr(q_refd_1, -1.0, 1.0)
		self.q_refd[ti] = q_refd_1
			
		# q_refdd
		if ti_0 > 2:
			q_refdd_1 = (q_refd_1 - self.q_refd[ti-1]) / dt
			#print 'r',r.s,r.v
		else:
			q_refdd_1 = np.zeros(3)
		
		self.q_refdd[ti] = q_refdd_1
		
		# omega ref
		self.omega_ref[ti] = self.q_refd[ti]
		
		# omega error
		self.e2[ti] = self.omega_ref[ti] - self.c.omega[ti]
	
		# e1 mag d	
		if ti_0 > 0:
			self.e1_mag_d[ti] = (vec.mag(self.e1[ti].v) - vec.mag(self.e1[ti-1].v)) / dt
		
		# check objective
		if ti_0 > 0:
			if self.obj:
				if self.obj.mode == control.ObjMode.normal:
					if (self.e1_mag_d[ti] < 0.0) and (self.e1_mag_d[ti] > -0.001):
						if (2.0 * math.asin(vec.mag(self.e1[ti].v))) < self.obj.thresh:
							self.obj.flag_complete = True
		
				
		
		# extras
		
		def prin():
			print 'q_ref_1 ',q_ref_1.v
			print 'q_ref_0 ',q_ref_0.v
			print 'r       ',r.v
			print 'q_refd_n',q_refd_1

		
		if np.any(q_refd_1 > 1.0):
			prin()

		ver = False
		#ver = True
		if ver:		
			prin()
		
	def get_tau_RB(self, ti, ti_0):
		# require error values
		self.step(ti, ti_0)
		
		q_ref = self.q_ref[ti]
		
		q = self.c.q[ti]
		
		e1 = self.e1[ti]
		
		omega = self.c.omega[ti]
		I = self.c.I
		
		omegad = np.dot(self.C1, e1.v) + np.dot(self.C2, self.e2[ti]) + self.q_refdd[ti]
		#tau_RB = omegad
		tau_RB = np.dot(I, omegad) + np.cross(omega, np.dot(I, omega))
		
		self.tau_RB[ti] = tau_RB
		
		if any(np.isnan(tau_RB)):
			print q_ref.s,q_ref.v
			print q.s,q.v
			print e1.s, e1.v
			
			
			raise ValueError('tau_RB nan')
		
		return tau_RB
		
	def plot(self):
		self.plot_q()
		self.plot_omega()
		self.plot_qrefdd()
	def plot_q(self):
		fig = pl.figure()

		N = self.c.ti

		t = self.c.t[:N]
		
		ax = fig.add_subplot(111)
		ax.plot(self.c.t, self.tau_RB)
		ax.set_xlabel('t')
		ax.set_ylabel('tau_RB')
		ax.legend(['x','y','z'])
		
		# orientation
		
		q = np.zeros((N,4))
		q_ref = np.zeros((N,4))
		
		print np.shape(q)
		
		for i in range(N):
			q[i,1:4] = self.c.q[i].v
			q[i,0] = self.c.q[i].s

			if self.q_ref[i]:
				q_ref[i,1:4] = self.q_ref[i].v
				q_ref[i,0] = self.q_ref[i].s
		
		fig = pl.figure()
		ax = fig.add_subplot(111)

		print np.shape(q[:,0])

		#ax.plot(self.c.t, q[:,0],'b-')
		ax.plot(t, q[:N,1],'g-')
		ax.plot(t, q[:N,2],'r-')
		ax.plot(t, q[:N,3],'c-')
		
		#ax.plot(self.c.t, q_ref[:,0],'b--')
		ax.plot(t, q_ref[:N,1],'g--')
		ax.plot(t, q_ref[:N,2],'r--')
		ax.plot(t, q_ref[:N,3],'c--')
		
		ax.set_xlabel('t')
		ax.set_ylabel('q')
		#ax.legend(['a','b','c','d','a','b','c','d'])
		ax.legend(['b','c','d','b_ref','c_ref','d_ref'])

	def plot_omega(self):
		t = self.c.t
		
		fig = pl.figure()
		ax = fig.add_subplot(111)
		
		#print np.shape(q[:,0])
		omega = self.c.omega
		q_refd = self.q_refd
		
		ax.plot(t, omega[:,0],'b-')
		ax.plot(t, omega[:,1],'g-')
		ax.plot(t, omega[:,2],'r-')
		
		ax.plot(t, q_refd[:,0],'b--')
		ax.plot(t, q_refd[:,1],'g--')
		ax.plot(t, q_refd[:,2],'r--')
		
		ax.set_xlabel('t')
		ax.set_ylabel('omega')
		
		ax.legend(['x','y','z','x_q_refd','y_q_refd','z_q_refd'])
	
	def plot_qrefdd(self):
		t = self.c.t
		
		fig = pl.figure()
		ax = fig.add_subplot(111)
		
		q_refdd = self.q_refdd
		
		ax.plot(t, q_refdd[:,0],'b--')
		ax.plot(t, q_refdd[:,1],'g--')
		ax.plot(t, q_refdd[:,2],'r--')
		
		ax.set_xlabel('t')
		ax.set_ylabel('q_refdd')
		
		ax.legend(['x','y','z'])


