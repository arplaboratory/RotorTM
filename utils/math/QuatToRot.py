import numpy as np

def QuatToRot(q):
    # converts a quaternion to a rotation matrix

    q = np.divide(q, np.sqrt(np.sum(np.power(q, 2))))

    qahat = np.zeros((3,3), dtype=float)
    qahat[0,1] = -q[3, 0]
    qahat[0,2] = q[2, 0]
    qahat[1,2] = -q[1, 0]
    qahat[1,0] = q[3, 0]
    qahat[2,0] = -q[2, 0]
    qahat[2,1] = q[1, 0]

    return (np.eye(3)+np.multiply(2, np.multiply(qahat, qahat))+np.multiply(2, np.multiply(q[0,0], qahat)))