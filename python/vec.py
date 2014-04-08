import numpy as np
import math

tol = 1e-6

e2 = np.array([0.,0.,1.])

def sign(x):
	return math.copysign(1.0,x)

def clamparr(x,minx,maxx):
	#x = np.array(x)
	x[x > maxx] = maxx
	x[x < minx] = minx
	return x

def clamp(x,minx,maxx):
	#x = np.array(x)
	if x > maxx:
		print 'max',maxx
		return maxx
	elif x < minx:
		print 'min',minx
		return minx
	else:
		return x

def clampz(x,v):
	return clamp(x,-v,v)

def magsq(a):
	return np.sum(np.square(a))
def mag(a):
	return math.sqrt(magsq(a))

def set_arb_comp(a,b):
	am = mag(a)
	ah = a / am

	bm = mag(b)
	bh = b / bm
	
	bm2 = am / np.dot(ah,bh)
	
	return bm2

def angle(a,b,up,ver = False):
	# rotation between a and b
	a_norm = np.linalg.norm(a)	
	b_norm = np.linalg.norm(b)
	
	if a_norm == 0 or b_norm == 0:
		return None,None,None,None
	
	d = np.dot(a,b)
	
	#d_angle = math.acos(d / a_norm / b_norm)
	
	c = np.cross(a,b)
	
	d_up = np.dot(c,up)
	
	c_norm = np.linalg.norm(c)
	
	
	
	c_angle = math.asin(c_norm / a_norm / b_norm)
	
	if d < 0:
		c_angle = math.pi - c_angle

	if d_up < 0:
		c_angle = -c_angle
	

	if ver:
		print 'd      ',d
		print 'd_up   ',d_up
		print 'c_angle',c_angle
		print 'c_norm ',c_norm
	
	
	return c, c_norm, c_angle, d_up

def equal(a,b):
	return math.fabs(a-b) < tol

def vec_to_euler(z, y = None):
	a = math.sqrt(1.0 - z[2]**2)
	
	if equal(a,0.0):
		if y:
			phi = math.acos(-y[0])
		else:
			phi = 0.0
		
		theta = 0.0
		psi = 0.0
	else:
		b = -z[1]/a
		
		if math.fabs(b) > 1.0:
			if equal(b, 1.0):
				phi = 0
			elif equal(b, -1.0):
				phi = math.pi
			else:
				print 'z',z
				print 'a',a
				print 'b',b
				raise ValueError('math domain error')
		else:
			phi = math.acos(b)
		
		theta = math.acos(z[2])
		if y:
			psi = math.acos(y[2] / a)
		else:
			psi = 0.0


	if phi > (math.pi / 2.0):
		phi = phi - math.pi
		theta = -theta
		#psi = math.pi + psi

	
	return np.array([phi, theta, psi])

def normalize(v):
	m = mag(v)
	if m == 0.0:
		raise ValueError('magnitude zero')
	return v / m

if __name__ == '__main__':
	a = np.array([1.,0.,0.])
	
	angle(a,np.array([ 1., 1.,0.]),e2, ver=True)
	angle(a,np.array([-1., 1.,0.]),e2, ver=True)
	angle(a,np.array([-1.,-1.,0.]),e2, ver=True)
	angle(a,np.array([ 1.,-1.,0.]),e2, ver=True)





