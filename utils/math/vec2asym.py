import numpy as np

def vec2asym(vec):
    row = vec.shape[0]
    col = vec.shape[1]
    if (row==3 and col==1) or (row==1 and col==3):
        return np.array([[0,-vec[2,0], vec[1,0]], [vec[2,0], 0, -vec[0]], [-vec[1], vec[0], 0]])