import numpy as np
# test passed


def RotToRPY_ZXY(R):
    phi = np.arcsin(R[1, 2])
    psi = np.arctan2(-R[1, 0] / np.cos(phi), R[1, 1] / np.cos(phi))
    theta = np.arctan2(-R[0, 2] / np.cos(phi), R[2, 2] / np.cos(phi))

    return phi, theta, psi
