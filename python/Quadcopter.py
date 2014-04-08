import pylab as pl
import numpy as np
import math
import vec
import quaternion as qt

#Inverse[{{1,0,-sin(y)}, {0, cos(x), cos(y)*sin(x)}, {0, -sin(x), cos(y)*cos(x)}}]
#Inverse[{{1, 0, -Sin[y]}, {0, Cos[x], Cos[y] Sin[x]}, {0, -Sin[x], Cos[y] Cos[x]}}]

#{{0, Cos[f[t]] Tan[g[t]] f'[t] + Sec[g[t]]^2 Sin[f[t]] g'[t], -(Sin[f[t]] Tan[g[t]] f'[t]) + Cos[f[t]] Sec[g[t]]^2 g'[t]},
#{0, -(Sin[f[t]] f'[t]), -(Cos[f[t]] f'[t])},
#{0, Cos[f[t]] Sec[g[t]] f'[t] + Sec[g[t]] Sin[f[t]] Tan[g[t]] g'[t], -(Sec[g[t]] Sin[f[t]] f'[t]) + Cos[f[t]] Sec[g[t]] Tan[g[t]] g'[t]}}

class Quad:
	def __init__(self, t):

		# physical constants
		self.m	= 1.0	# mass (kg)
		
		self.L	= 0.25	# arm length (m)
		
		R 	= 0.05	# prop radius (m)
		Asw	= math.pi * R**2
		
		
		rho 	= 1.28	# (kg/m3) air density
		

		CD 	= 1.0	# dimensionless const
		A	= 0.05 * 0.01	# prop cross-sectional area (m2)
		
				
		Kv	= 1450	# back EMF (RPM / V)
		
		
		Kv = 1.0 / Kv * 0.1047
		
		Kt	= Kv
		
		Ktau	= 0.5
		
		
		self.k = Kv * Ktau * math.sqrt(rho * Asw)
		self.b = 0.5 * R**3 * rho * CD * A
		
		
		self.t = t
		self.gravity = np.array([0,0,-9.81])
		
		self.N = len(t)
		
		self.I = np.identity(3)
		self.Iinv = np.linalg.inv(self.I)
		
			
		# state variables
		self.q = np.empty(self.N, dtype=object)
		self.q[0] = qt.Quat()
		
		self.theta = np.zeros((self.N,3))
		self.omega = np.zeros((self.N,3))
		
		self.x = np.zeros((self.N,3))
		self.v = np.zeros((self.N,3))
		
		# constants

		# motor speed
		self.gamma = np.zeros((self.N,4))
		
		# matrices
		self.A1 = np.array([
				[self.L*self.k,		0,		-self.L*self.k,	0],
				[0,			self.L*self.k,	0,		-self.L*self.k],
				[self.b,		-self.b,	self.b,		-self.b]])		

		self.A2 = np.array([self.k,self.k,self.k,self.k,])
		
		self.A4 = np.append(self.A1, np.reshape(self.A2,(1,4)), 0)

		#print self.A4
		self.A4inv = np.linalg.inv(self.A4)

		#print 'q',self.q._q
		#print 'w',self.w
	def get_A3(self, ti):
		q = self.q[ti]._q
		x = q[0]
		y = q[1]
		z = q[2]
		w = q[3]
		A3 = np.array([
			[-x,-y,-z],
			[ w,-z, y],
			[ z, w,-x],
			[-y, x, w]])
		return A3
	def get_A5(self, ti):
		return np.linalg.inv(self.get_A5inv(ti))
	def get_A5inv(self, ti):
		p = self.theta[ti,0]
		t = self.theta[ti,1]
		
		st = math.sin(t)
		ct = math.cos(t)
		sp = math.sin(p)
		cp = math.cos(p)
		
		A5inv = np.array([
				[1, 0, -st],
				[0, cp, ct * sp],
				[0, -sp, ct * cp]])
		return A5inv
	def get_A5d(self, ti):
		p = self.theta[ti,0]
		t = self.theta[ti,1]
		
		thetad = self.get_thetad(ti)
		
		pd = thetad[0]
		td = thetad[1]
		
		st = math.sin(t)
		ct = math.cos(t)
		sp = math.sin(p)
		cp = math.cos(p)
		
		tant = math.tan(t)
		sect = 1.0 / ct
		
		A5d = np.array([
			[0,	cp * tant * pd + sect**2 * sp * td,		-sp * tant * pd + cp * sect**2 * td],
			[0,	-sp * pd,					-cp * pd],
			[0,	cp * sect * pd + sect * sp * tant * td,		-sect * sp * pd + cp * sect * tant * td]])
		
		return A5d
	def get_A6(self, ti):
		theta  = self.theta[ti]
		sp = math.sin(theta[0])
		st = math.sin(theta[1])
		ss = math.sin(theta[2])
		cp = math.cos(theta[0])
		ct = math.cos(theta[1])
		cs = math.cos(theta[2])
		R = np.array([
				[cp * cs - ct * sp * ss,	-cs * sp - cp * ct * ss,	st * ss],
				[ct * cs * sp + cp * ss,	cp * ct * cs - sp * ss,		-cs * st],
				[sp * st,			cp * st,			ct]])
		return R
	def get_thetad(self, ti):
		A5 = self.get_A5(ti)
		omega = self.omega[ti]
		thetap = np.dot(A5, omega)
		return thetap
	def get_tau_body(self):
		tau = self.get_tau_rotor_body()
		return T
	def get_tau_rotor_body(self, ti):
		gamma = self.gamma[ti]

		if any(np.isnan(gamma)):
			raise ValueError('gamma nan')

		tau = np.dot(self.A1, gamma)
		return tau
	def get_force_rotor_body(self, ti):
		T = np.zeros(3)
		T[2] = np.dot(self.A2, self.gamma[ti])
		return T
	def get_force_drag_body(self, ti):
		return np.zeros(3)
	def get_force_drag(self, ti):
		return np.dot(self.get_A6(ti), self.get_force_drag_body(ti))
	def get_force(self, ti):
		q = self.q[ti]
		
		f_g = self.gravity
		
		f_B = self.get_force_rotor_body(ti) + self.get_force_drag_body(ti)
		
		if any(np.isnan(f_B)):
			raise ValueError('f_B nan')
		
		f = f_g + q.conj().rotate(f_B)
		
		ver = False
		if ver:	
			print 'A6 ',A6
			print 'f_g',f_g
			print 'f_B',f_B
			print 'f  ',f
		
		return f
		
	
	def step(self, ti):
		self.ti = ti

		dt = self.t[ti] - self.t[ti-1]
		
		# rotation
		omega  = self.omega[ti-1]
		q = self.q[ti-1]		

		tau = self.get_tau_rotor_body(ti-1)
				
		omegad = np.dot(self.Iinv, tau - np.cross(omega, np.dot(self.I, omega)))

		#theta  = self.theta[ti-1]
		#thetap = self.get_thetad(ti-1)
		
		omega_n = omega + omegad * dt
		
		#self.theta[ti] = theta + thetap * dt
		
		omega_n_magn = vec.mag(omega_n)
		
		#print omega_magn
		if omega_n_magn == 0.0:
			r = qt.Quat()
		else:
			omega_n_norm = omega_n / omega_n_magn
			r = qt.Quat(theta = omega_n_magn * dt, v = omega_n_norm)
		
		qn = r * q
		
		
		self.omega[ti] = omega_n
		self.q[ti] = qn

		ver = False
		if ver:		
			print 'tau    ',tau
			print 'omegad ',omegad
			print 'omega_n',omega_n
			print 'r      ',r.s,r.v

		# position
		f = self.get_force(ti-1)
		
		if any(np.isnan(f)):
			raise ValueError('f nan')
		
		x = self.x[ti-1]
		v = self.v[ti-1]
		
		a = f / self.m
		
		
		vn = v + a * dt
		xn = x + vn * dt
		
		self.x[ti] = xn 
		self.v[ti] = vn
		
		
		if any(np.isnan(vn)):
			raise ValueError('v nan')
		if any(np.isnan(xn)):
			raise ValueError('x nan')

		#print self.x

	def plot3(c):
		fig = pl.figure()
		ax = fig.gca(projection='3d')
		
		x = c.x[:,0]
		y = c.x[:,1]
		z = c.x[:,2]
		
		s = (np.max(np.max(c.x)) - np.min(np.min(c.x))) / 2.0
		
		ax.plot(x,y,z,'o')
		
		rx = (np.max(x)+np.min(x))/2.0
		ry = (np.max(y)+np.min(y))/2.0
		rz = (np.max(z)+np.min(z))/2.0
	
		ax.set_xlim3d(rx-s,rx+s)
		ax.set_ylim3d(ry-s,ry+s)
		ax.set_zlim3d(rz-s,rz+s)
		
	def plot(self):
		#self.plot_x()
		#self.plot_v()
		pass
	def plot_x(self):
		fig = pl.figure()

		t = self.t
		x = self.x[:,0]
		y = self.x[:,1]
		z = self.x[:,2]
		
		#print np.shape(t)
		#print np.shape(x)

		#print t
		#print x

		ax = fig.add_subplot(111)
		ax.set_xlabel('t')
		ax.set_ylabel('x')
		ax.plot(t,x,'b')#,t,y,'g',t,z,'r')
		ax.plot(t,y,'g')
		ax.plot(t,z,'r')
		
		ax.legend(['x','y','z'])
		
	def plot_theta(self):
		fig = pl.figure()
		
		ax = fig.add_subplot(111)
		ax.set_xlabel('t')
		ax.set_ylabel('theta')
		ax.plot(self.t,self.theta)
		
		ax.legend(['phi','theta','psi'])
		
	def plot_v(self):
		fig = pl.figure()
		
		ax = fig.add_subplot(111)
		ax.set_xlabel('t')
		ax.set_ylabel('v')
		ax.plot(self.t,self.v)
		ax.legend(['x','y','z'])
		
	def write(self,stride = 1):
		f1 = open("q.txt","w")
		f2 = open("x.txt","w")
		
		for ti in range(0,self.ti,stride):
			q = self.q[ti]
			f1.write("{0},{1},{2},{3}\n".format(q.s,q.v[0],q.v[1],q.v[2]))

			x = self.x[ti]
			f2.write("{0},{1},{2}\n".format(x[0],x[1],x[2]))

		f1.close()
		f2.close()


		
