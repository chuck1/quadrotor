import numpy as np
import pylab as pl

def rootlocus(c):
	r = np.roots(c)
	print r

	pl.plot(np.real(r),np.imag(r),'o')
	pl.grid('on')
	



#r = [-1,-1,-1,-1]
r = [-0.5]*5

c = np.poly(r)

print c

rootlocus(c)

pl.show()

