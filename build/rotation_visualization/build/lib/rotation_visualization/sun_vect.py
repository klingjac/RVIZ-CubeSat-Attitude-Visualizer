import numpy as np

def compute_vect(I1, I2, I3, I21, I22, I23):
    s10 = np.sin(np.radians(10))
    c10 = np.cos(np.radians(10))
    s30 = np.sin(np.radians(30))
    c30 = np.cos(np.radians(30))

    # Triclops 1: +Z aligned, +X aligned diode 2
    n1 = np.array([-s30*s10, c30*s10, c10])
    n2 = np.array([s10, 0, c10])
    n3 = np.array([-s30*s10, -c30*s10, c10])
    # Triclops 2: -Z aligned, -X aligned diode 2
    n4 = np.array([s30*s10, c30*s10, c10])
    n5 = np.array([-s10, 0, c10])
    n6 = np.array([s30*s10, -c30*s10, c10])

    H = np.vstack((n1,n2,n3))

    # Stack the input current values and remove values below the noise floor
    y = np.vstack((I1,I2,I3))
    y[y < 0.1] = 0
    #print(y)

    R = 7e-11 * np.identity(3) #Measurement covariance matrix 

    S = np.linalg.inv(H.T @ np.linalg.inv(R) @ H) @ H.T @ np.linalg.inv(R) @ y

    S = S / np.linalg.norm(S)
    # print(f"1: {I21}, 2: {I22}, 3: {I23}")
    print(f"vect: {S.flatten()}")
    return S