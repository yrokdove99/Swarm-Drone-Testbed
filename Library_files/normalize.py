import numpy as np
import numpy.linalg as np_li
import math

def normalized_vector(v:np.array)->np.array:
    # normalized_v = v / np.sqrt(np.sum(v**2))
    try:
        norm_v = v / np_li.norm(v)
    except Exception as e:
        norm_v = v # if np_la.norm(v) is zero
        
    return norm_v

#m = np.arange(6).reshape(2,3)
def normalized_matrix(m:np.array):
    row, col = 0, 1

    if m.shape[0] != 2:
        raise Exception('matrix가 vector matrix가 아닙니다. 2xN 형식의 matrix로 만드십시오.')
    dvi = m.sum(axis=row)
    vx = m[0,:] / dvi # size: (1, N)
    vy = m[1,:] / dvi # size: (1, N)
    mat = np.array([vx, vy])
    return mat


def ceil_from_digits(n, digit):
    dig = 10 ** digit
    n = math.ceil(dig*n)
    n = n/dig
    return n