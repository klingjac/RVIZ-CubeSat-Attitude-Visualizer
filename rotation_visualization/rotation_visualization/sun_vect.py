import numpy as np
import math
def compute_vect(I1, I2, I3, I21, I22, I23, pannels):
    s10 = np.sin(np.radians(10))
    c10 = np.cos(np.radians(10))
    s30 = np.sin(np.radians(30))
    c30 = np.cos(np.radians(30))

    # Triclops 1: +Z aligned, +X aligned diode 2
    n1 = np.array([-s30*s10, -c30*s10, c10])
    n2 = np.array([s10, 0, c10])
    n3 = np.array([-s30*s10, c30*s10, c10])
    # Triclops 2: -Z aligned, -X aligned diode 2
    n4 = np.array([s30*s10, c30*s10, -c10])
    n5 = np.array([-s10, 0, -c10])
    n6 = np.array([s30*s10, -c30*s10, -c10])

    n7 = np.array([1,0,0])
    n8 = np.array([0,1,0])
    n9 = np.array([-1,0,0])
    n10 = np.array([0,-1,0])

    H = np.vstack((n1,n2,n3,n4,n5,n6,n7,n8,n9,n10))

    # Stack the input current values and remove values below the noise floor
    p1 = pannels[0]
    p2 = pannels[1]
    p3 = pannels[2]
    p4 = pannels[3]
    y = np.vstack((I1,I2,I3,I21,I22,I23,p1,p2,p3,p4))
    y[y < 0.1] = 0
    #print(y)

    R = 7e-11 * np.identity(10) #Measurement covariance matrix 

    S = np.linalg.inv(H.T @ np.linalg.inv(R) @ H) @ H.T @ np.linalg.inv(R) @ y

    S = S / np.linalg.norm(S)
    if math.isnan(S[0]):
        S = np.array([0,0,1])
    return S
    #print(S)
