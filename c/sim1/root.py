import numpy as np
import pylab as pl

def rootlocus(c):
	r = np.roots(c)
	print r

	pl.plot(np.real(r),np.imag(r),'o')
	pl.grid('on')
	



#r = [-1,-1,-1,-1]
r = [-0.5]*4

c = np.poly(r)

print c

d = [
		3.562152,
		8.248242,
		9.297224,
		4.464712,
		-0.012777]

d = c * 2.0

print d

rootlocus(c)
rootlocus(d)

pl.show()

