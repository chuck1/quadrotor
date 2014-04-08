import numpy as np

a = np.ones(3)

print 'a',a

b = np.reshape(a,(1,3))

print 'b',b

c = np.append(a,b,0)

print c


