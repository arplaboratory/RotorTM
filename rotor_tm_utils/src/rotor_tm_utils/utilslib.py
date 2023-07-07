from types import DynamicClassAttribute
import numpy as np
import numpy.linalg as LA


def addnoise(state):
    return 0.0


def vec2asym(vec):
    if (isinstance(vec, np.ndarray)):
        if len(vec.shape) == 1:
            row = vec.shape[0]
            col = 0
        elif len(vec.shape) == 2:
            row = vec.shape[0]
            col = vec.shape[1]
    elif isinstance(vec, list):
        row = len(vec)
        col = 0
    else:
        raise Exception("The vector type not list or numpy array")

    if row == 3:
        if col == 0:
            mat = np.array([[0, -vec[2], vec[1]],
                            [vec[2], 0, -vec[0]],
                            [-vec[1], vec[0], 0]])
        elif col == 1:
            mat = np.array([[0, -vec[2][0], vec[1][0]],
                            [vec[2][0], 0, -vec[0][0]],
                            [-vec[1][0], vec[0][0], 0]])
    else:
        raise Exception("The vector shape is not 3")

    return mat


def QuatToRot(quat):
    if (isinstance(quat, np.ndarray)):
        q = quat
    elif isinstance(quat, list):
        q = np.array(quat)
    else:
        raise Exception("The vector type not list or numpy array")

    q = q / np.sqrt(np.sum(q**2))

    qahat = np.zeros((3, 3))
    qahat[0, 1] = -q[3]
    qahat[0, 2] = q[2]
    qahat[1, 2] = -q[1]
    qahat[1, 0] = q[3]
    qahat[2, 0] = -q[2]
    qahat[2, 1] = q[1]

    R = np.eye(3) + 2 * np.matmul(qahat, qahat) + 2 * q[0] * qahat

    return R


def quat_dot(quat, omg):

    qW = quat[0]
    qX = quat[1]
    qY = quat[2]
    qZ = quat[3]

    p = omg[0]
    q = omg[1]
    r = omg[2]

    K_quat = 2
    quaterror = 1 - (qW ** 2 + qX ** 2 + qY ** 2 + qZ ** 2)
    # qLdot = - 1 / 2 * np.array([[0,- p,- q,- r],[p,0,- r,q],[q,r,0,- p],[r,- q,p,0]]) * quat + K_quat * quaterror * quat
    qLdot = 1 / 2 * np.matmul(np.array([[0, - p, - q, - r], [p, 0, r, -q], [q, -r, 0, p], [r, q, -p, 0]]), quat) + K_quat * quaterror * quat

    return qLdot


def expSO3(w):
    # Exponential of SO3 to get the rotation matrix

    if (isinstance(w, type(np.array([0])))):
        if (w.shape == (3, 1) or w.shape == (1, 3) or w.shape == (3,)):

            w = w.flatten()

            one_6th = 1.0 / 6.0
            one_20th = 1.0 / 20.0

            theta = LA.norm(w)
            theta_sq = theta * theta

            # Use a Taylor series expansion near zero. This is required for
            # accuracy, since sin t / t and (1-cos t)/t^2 are both 0/0.
            if (theta_sq < 1e-8):
                A = 1.0 - one_6th * theta_sq
                B = 0.5
            else:
                if (theta_sq < 1e-6):
                    B = 0.5 - 0.25 * one_6th * theta_sq
                    A = 1.0 - theta_sq * one_6th * (1.0 - one_20th * theta_sq)
                else:
                    inv_theta = 1.0 / theta
                    A = np.sin(theta) * inv_theta
                    B = (1 - np.cos(theta)) * (inv_theta * inv_theta)

            result = rodrigues_so3_exp(w, A, B)
            return result

        else:
            print("The size of the input variable is not 3, intead it is, ", w.shape, ".")
            print("Returning empty array.")
            return np.array([])

    else:
        print("The type of input variable is not numpy array, instead it is ", type(w), ".")
        print("Returning empty array.")
        return np.array([])


def rodrigues_so3_exp(w, A, B):

    R = np.zeros((3, 3))

    wx2 = w[0] * w[0]
    wy2 = w[1] * w[1]
    wz2 = w[2] * w[2]

    R[0, 0] = 1.0 - B * (wy2 + wz2)
    R[1, 1] = 1.0 - B * (wx2 + wz2)
    R[2, 2] = 1.0 - B * (wx2 + wy2)

    a = A * w[2]
    b = B * (w[0] * w[1])

    R[0, 1] = b - a
    R[1, 0] = b + a

    a = A * w[1]
    b = B * (w[0] * w[2])

    R[0, 2] = b + a
    R[2, 0] = b - a

    a = A * w[0]
    b = B * (w[1] * w[2])

    R[1, 2] = b - a
    R[2, 1] = b + a

    return R


def toint(A):
    B = np.zeros(A.shape, dtype=int)
    # take an array, converts all elements to integer
    for i in range(np.max(A.shape)):
        B[i] = int(A[i])
    return B
