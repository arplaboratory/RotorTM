#! /usr/bin/env python

import numpy as np
import numpy.linalg as LA

# test passed


def allocate_time(path, max_vel, max_acc):
    pathlength = path.shape[0]

    distance = path[1:pathlength, :] - path[0:pathlength - 1, :]
    distance = np.transpose(LA.norm(distance, axis=1))

    time_segment = np.multiply(3.0 * (distance * 2 / max_vel), (1 + np.exp(-2 * distance / max_vel) * 6.5 * max_vel / max_acc))

    return time_segment
