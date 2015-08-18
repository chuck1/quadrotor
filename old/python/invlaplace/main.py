import numpy as np

def fun(t,a):
	
	for i in range(len(a)):
		for j in range(i+1,len(a)):
			if a[i] == a[j]:
				print i,j
				del a[j]
				del a[i]
	
	y = np.zeros(np.shape(t))
	for b in a:
		y += np.exp(b * t)


t = np.arange(0,100) * 0.001

fun(t,[-1,-1])


