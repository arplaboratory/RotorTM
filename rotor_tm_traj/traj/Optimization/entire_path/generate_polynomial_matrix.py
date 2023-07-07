#! /usr/bin/env python

import numpy as np
# test passed


def generate_polynomial_matrix(max_exponent, max_diff, var):
    matrix = np.zeros((0, max_exponent + 1), dtype=float)
    for i in range(0, max_diff + 1):
        polynomial = np.array([[]])
        for j in range(0, max_exponent + 1):
            exponent = j - i
            if exponent >= 0:
                term = var**exponent
            else:
                term = 0
            polynomial = np.append(polynomial, np.array([[term]]), axis=1)
        matrix = np.append(matrix, polynomial, axis=0)
    return matrix
