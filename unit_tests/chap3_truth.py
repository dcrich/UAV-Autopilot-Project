"""
Expected values for mav_dynamics in chap3
"""
import numpy as np

# _derivatives() = x_dot  <- [north_dot, east_dot, down_dot, u_dot, v_dot, w_dot, e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]
x_dot_fx = np.zeros((13,1),dtype=float)
x_dot_fx[0,0] = 25.0
x_dot_fx[3,0] = 0.90909091

x_dot_fy = np.zeros((13,1),dtype=float)
x_dot_fy[0,0] = 25.0
x_dot_fy[4,0] = 0.90909091

x_dot_fz = np.zeros((13,1),dtype=float)
x_dot_fz[0,0] = 25.0
x_dot_fz[5,0] = 0.90909091

x_dot_Mx = np.zeros((13,1),dtype=float)
x_dot_Mx[0,0] = 2.50000000e+01
x_dot_Mx[10,0] = 1.22525166e-01
x_dot_Mx[12,0] = 8.38660032e-03

x_dot_My = np.zeros((13,1),dtype=float)

x_dot_Mz = np.zeros((13,1),dtype=float)


# update() = state  <-  [north, east, altitude, phi, theta, psi, p, q, r, Va, Vg]
state_fx = np.zeros((11,1),dtype=float)
state_fx[0,0] = 0.25004545
state_fx[3,0] = 25.00909091
state_fx[6,0] = 1.0

state_fy = np.zeros((11,1),dtype=float)
state_fy[0,0] = 0.25
state_fy[1,0] = 4.54545455e-05
state_fy[3,0] = 25.0
state_fy[4,0] = 9.09090909e-03
state_fy[6,0] = 1.0

state_fz = np.zeros((11,1),dtype=float)
state_fz[0,0] = 2.50000000e-01
state_fz[2,0] = 4.54545455e-05
state_fz[3,0] = 2.50000000e+01
state_fz[5,0] = 9.09090909e-03
state_fz[6,0] = 1.0

state_Mx = np.zeros((11,1),dtype=float)
state_Mx = np.array([ 2.50000000e-01]
                    [ 1.77635163e-20]
                    [-2.67596249e-14]
                    [ 2.50000000e+01]
                    [-1.04832504e-05]
                    [ 1.67176432e-11]
                    [ 1.00000000e+00]
                    [ 3.06312914e-06]
                    [-3.07878132e-13]
                    [ 2.09665008e-07]
                    [ 1.22525166e-03]
                    [-2.46302506e-10]
                    [ 8.38660032e-05])

state_My = np.zeros((11,1),dtype=float)

state_Mz = np.zeros((11,1),dtype=float)
