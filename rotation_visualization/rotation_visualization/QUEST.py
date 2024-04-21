#Written by Brian Leung

"""
PURPOSE:
    Implementation of the quest algorithm.
PARAMETERS:
    body_vecs: Nx3 numpy array of unit length body measurement vectors, where N>=2
    weights: Nx1 numpy array of weights, corresponding to each of the unit body vectors 
    inertial_vecs: Nx3 numpy array corresponding inertial vectors
OUTPUT:
    Quaternion of current attitude <q0 q1 q2 q3>, where q0 is the scalar term
    (It'll just be a 1x4 numpy array, since numpy doesnt support quaternions natively.)
"""

import numpy as np
import rotation_visualization.eig_helper as eh
from math import sqrt
import time

def quest(body_vecs,weights,inertial_vecs,precision=0.000001):

    #Ensuring proper vector lengths of all inputs
    if (body_vecs.shape[1] != 3 or inertial_vecs.shape[1] != 3 or weights.shape[1] != 1):
        raise ValueError("Check the dimensions on your arrays.")
    #Ensuring same number of inertial and body vectors
    if (body_vecs.shape != inertial_vecs.shape):
        raise ValueError("Unequal numbers of inertial and body vectors.")
    #Ensuring same number of weights and body vectors
    if (body_vecs.shape[0] != weights.shape[0]):
        raise ValueError("Unequal numbers of weights and body vectors.")
    vec_count = body_vecs.shape[0]

    #Just ensures the vectors aren't stuck as ints.
    body_vecs = body_vecs*1.0
    inertial_vecs = inertial_vecs*1.0

    ##DETERMINING APPROXIMATE EIGENVALUE
    eig_guess = weights.sum()
    
    #Calculating K matrix:
    B = np.zeros((3,3))
    for i in range(vec_count):
        B += weights[i]*body_vecs[i,:].reshape(3,1).dot(inertial_vecs[i,:].reshape(1,3))

    Z = np.array([[B[1,2]-B[2,1]],[B[2,0]-B[0,2]],[B[0,1]-B[1,0]]])
    sigma = B[0,0] + B[1,1] + B[2,2] #trace of B
    S = B+B.T
    K = np.zeros((4,4))
    K[0,0]=sigma
    K[1:,0]=Z.reshape(3)
    K[0,1:]=Z.reshape(3)
    K[1:,1:]=S-sigma*np.identity(3)
    
    ##NEWTON RAPHSON TO FIND ACTUAL EIGENVALUE
    c_l3 = eh.l3_coeff(K)
    c_l2 = eh.l2_coeff(K)
    c_l1 = eh.l1_coeff(K)
    c_l0 = eh.l0_coeff(K)
    while (abs(eh.chr_eq(eig_guess,c_l0,c_l1,c_l2,c_l3)) >= precision): #arbitrary precision
        eig_guess -= eh.chr_eq(eig_guess,c_l0,c_l1,c_l2,c_l3)/eh.diff_chr_eq(eig_guess,c_l1,c_l2,c_l3)

    ##SINGULARITY HANDLING
    #Will only trigger if the pre_crp_mat is singular. Should only recurse once.
    #Could this check be done somewhat earlier in the program, 
    #so we don't have to redo everything?
    pre_crp_mat = (eig_guess+sigma)*np.identity(3)-S
    if np.linalg.det(pre_crp_mat) == 0:
        #This alternate body frame is completely arbitrary.
        rot_mat = np.array([
            [0,0,1],
            [1,0,0],
            [0,1,0]])
        rot_quat = np.array([0.5,0.5,0.5,0.5])

        for i in range(vec_count):
            body_vecs[i,:] = (rot_mat.dot(body_vecs[i,:].reshape(3,1))).reshape(3,)

        alt_quat = quest(body_vecs,weights,inertial_vecs)

        ##Rotates the alternate quaternion back to original
        rot_quat_mat = np.array([
            [rot_quat[0],-rot_quat[1],-rot_quat[2],-rot_quat[3]],
            [rot_quat[1], rot_quat[0], rot_quat[3],-rot_quat[2]],
            [rot_quat[2],-rot_quat[3], rot_quat[0], rot_quat[1]],
            [rot_quat[3], rot_quat[2],-rot_quat[1], rot_quat[0]]
        ])
        ##Rounding takes care of any minor precision issues
        return np.around(rot_quat_mat.dot(alt_quat),8)
    
    ##CALCULATE OUTPUT IN CRPs
    crp = np.linalg.inv(pre_crp_mat).dot(Z)
    
    ##CONVERT TO QUATERNIONS
    temp_quat = np.array([[1.0],[0.0],[0.0],[0.0]])
    temp_quat[1:,0] = crp.reshape(3)
    quat = 1/sqrt(1+(crp.T).dot(crp))*temp_quat

    return(quat)
