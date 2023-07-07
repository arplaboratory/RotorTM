import numpy as np


def RPYtoRot_ZXY(phi, theta, psi):
    # RPYtoRot_ZXY Converts roll, pitch, yaw to a body-to-world Rotation matrix
    #   The rotation matrix in this function is world to body [bRw] you will
    #   need to transpose this matrix to get the body to world [wRb] such that
    #   [wP] = [wRb] * [bP], where [bP] is a point in the body frame and [wP]
    #   is a point in the world frame
    #   written by Daniel Mellinger

    R = np.array([[np.cos(psi) * np.cos(theta) - np.sin(phi) * np.sin(psi) * np.sin(theta), np.cos(theta) * np.sin(psi) + np.cos(psi) * np.sin(phi) * np.sin(theta), -np.cos(phi) * np.sin(theta)],
                  [-np.cos(phi) * np.sin(psi), np.cos(phi) * np.cos(psi), np.sin(phi)],
                  [np.cos(psi) * np.sin(theta) + np.cos(theta) * np.sin(phi) * np.sin(psi), np.sin(psi) * np.sin(theta) - np.cos(psi) * np.cos(theta) * np.sin(phi), np.cos(phi) * np.cos(theta)]])

    return R
