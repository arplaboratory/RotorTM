#! /usr/bin/env python
from math import factorial
import numpy as np
# test passed
def generate_poly(max_exponent,max_diff,symbol):
	
	f=np.zeros((max_diff+1, max_exponent+1), dtype=float)
	
	for k in range(max_diff+1):
		for i in range(max_exponent+1):
			if (i - k) >= 0:
				f[k,i]  = factorial(i)*symbol**(i-k)/factorial(i-k)
			else:
				f[k,i] = 0
	return f