import numpy as np
import numpy.linalg as np_li
import math

def normalized_vector(v:np.array)->np.array:
    try:
        norm_v = v / np_li.norm(v)
    except Exception as e:
        norm_v = v
        
    return norm_v

def normalized_matrix(m:np.array):
    row, col = 0, 1

    if m.shape[0] != 2:
        raise Exception('matrix is not vector matrix, please make it matrix in 2xN format.')
    dvi = m.sum(axis=row)
    vx = m[0,:] / dvi 
    vy = m[1,:] / dvi 
    mat = np.array([vx, vy])
    return mat

def ceil_from_digits(n, digit):
    dig = 10 ** digit
    n = math.ceil(dig*n)
    n = n/dig
    return n