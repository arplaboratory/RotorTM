import numpy as np


def vee(ss):
    size = ss.shape[0] * ss.shape[1]
    ss_round = np.around(ss, decimals=10)
    if size == 4:
        if (ss_round[0, 1] == ss_round[1, 0]):
            vec = ss[0, 1]
    elif size == 9:
        if (ss_round[2, 1] == -ss_round[1, 2] and ss_round[0, 2] == -ss_round[2, 0] and ss_round[1, 0] == -ss_round[0, 1]):
            vec = np.array([[ss[2, 1]], [ss[0, 2]], [ss[1, 0]]])
    return vec
