import numpy
import quadprog

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    # qp_G = .5 * (P + P.T) + numpy.eye(P.shape[0])*(1e-120)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -numpy.vstack([A, G]).T
        qp_b = -numpy.vstack([b, h])
        qp_b = qp_b.reshape((qp_b.shape[0]))
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(P + numpy.eye(P.shape[0])*(1e-12), qp_a, qp_C, qp_b, meq)[0] 

