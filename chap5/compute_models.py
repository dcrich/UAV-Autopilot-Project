"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        2/4/2019 - RWB
"""
# from errno import EL2HLT
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.rotations import Euler2Quaternion, Quaternion2Euler
import parameters.aerosonde_parameters as MAV
from parameters.simulation_parameters import ts_simulation as Ts
from message_types.msg_delta import MsgDelta


def compute_model(mav, trim_state, trim_input):
    A_lon, B_lon, A_lat, B_lat = compute_ss_model(mav, trim_state, trim_input)
    Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, \
    a_V1, a_V2, a_V3 = compute_tf_model(mav, trim_state, trim_input)

    # write transfer function gains to file
    file = open('chap5/model_coef.py', 'w')
    file.write('import numpy as np\n')
    file.write('x_trim = np.array([[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]]).T\n' %
               (trim_state.item(0), trim_state.item(1), trim_state.item(2), trim_state.item(3),
                trim_state.item(4), trim_state.item(5), trim_state.item(6), trim_state.item(7),
                trim_state.item(8), trim_state.item(9), trim_state.item(10), trim_state.item(11),
                trim_state.item(12)))
    file.write('u_trim = np.array([[%f, %f, %f, %f]]).T\n' %
               (trim_input.elevator, trim_input.aileron, trim_input.rudder, trim_input.throttle))
    file.write('Va_trim = %f\n' % Va_trim)
    file.write('alpha_trim = %f\n' % alpha_trim)
    file.write('theta_trim = %f\n' % theta_trim)
    file.write('a_phi1 = %f\n' % a_phi1)
    file.write('a_phi2 = %f\n' % a_phi2)
    file.write('a_theta1 = %f\n' % a_theta1)
    file.write('a_theta2 = %f\n' % a_theta2)
    file.write('a_theta3 = %f\n' % a_theta3)
    file.write('a_V1 = %f\n' % a_V1)
    file.write('a_V2 = %f\n' % a_V2)
    file.write('a_V3 = %f\n' % a_V3)
    file.write('A_lon = np.array([\n    [%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f]])\n' %
    (A_lon[0][0], A_lon[0][1], A_lon[0][2], A_lon[0][3], A_lon[0][4],
     A_lon[1][0], A_lon[1][1], A_lon[1][2], A_lon[1][3], A_lon[1][4],
     A_lon[2][0], A_lon[2][1], A_lon[2][2], A_lon[2][3], A_lon[2][4],
     A_lon[3][0], A_lon[3][1], A_lon[3][2], A_lon[3][3], A_lon[3][4],
     A_lon[4][0], A_lon[4][1], A_lon[4][2], A_lon[4][3], A_lon[4][4]))
    file.write('B_lon = np.array([\n    [%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f]])\n' %
    (B_lon[0][0], B_lon[0][1],
     B_lon[1][0], B_lon[1][1],
     B_lon[2][0], B_lon[2][1],
     B_lon[3][0], B_lon[3][1],
     B_lon[4][0], B_lon[4][1],))
    file.write('A_lat = np.array([\n    [%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f]])\n' %
    (A_lat[0][0], A_lat[0][1], A_lat[0][2], A_lat[0][3], A_lat[0][4],
     A_lat[1][0], A_lat[1][1], A_lat[1][2], A_lat[1][3], A_lat[1][4],
     A_lat[2][0], A_lat[2][1], A_lat[2][2], A_lat[2][3], A_lat[2][4],
     A_lat[3][0], A_lat[3][1], A_lat[3][2], A_lat[3][3], A_lat[3][4],
     A_lat[4][0], A_lat[4][1], A_lat[4][2], A_lat[4][3], A_lat[4][4]))
    file.write('B_lat = np.array([\n    [%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f]])\n' %
    (B_lat[0][0], B_lat[0][1],
     B_lat[1][0], B_lat[1][1],
     B_lat[2][0], B_lat[2][1],
     B_lat[3][0], B_lat[3][1],
     B_lat[4][0], B_lat[4][1],))
    file.write('Ts = %f\n' % Ts)
    file.close()


def compute_tf_model(mav, trim_state, trim_input):
    # trim values
    mav._state = trim_state
    mav._update_velocity_data()
    Va_trim = mav._Va
    alpha_trim = mav._alpha
    phi, theta_trim, psi = Quaternion2Euler(trim_state[6:10])
    delta_e= trim_input.elevator
    delta_a = trim_input.aileron
    delta_r = trim_input.rudder
    delta_t = trim_input.throttle

    # define transfer function constants
    a_phi1 = -0.5 * MAV.rho * Va_trim**2 * MAV.S_wing * MAV.b**2 * MAV.C_p_p * 0.5 / Va_trim
    a_phi2 = 0.5 * MAV.rho * Va_trim**2 * MAV.S_wing * MAV.b * MAV.C_p_delta_a
    a_theta1 = MAV.rho * Va_trim**2 * MAV.S_wing * MAV.C_m_q * MAV.c**2 * 0.25 / (MAV.Jy * Va_trim)
    a_theta2 = MAV.rho * Va_trim**2 * MAV.c * MAV.S_wing * MAV.C_m_alpha * 0.5 / (MAV.Jy)
    a_theta3 = MAV.rho * Va_trim**2 * MAV.c * MAV.S_wing * MAV.C_m_delta_e * 0.5 / (MAV.Jy)

    # Compute transfer function coefficients using new propulsion model
    diffTp_wrt_Va = dT_dVa(mav, Va_trim, delta_t)
    diffTp_wrt_delta_t = dT_ddelta_t(mav, Va_trim, delta_t)
    a_V1 = MAV.rho * Va_trim * MAV.S_wing * (MAV.C_D_0 + MAV.C_D_alpha * alpha_trim + MAV.C_D_delta_e * delta_e) / MAV.mass - diffTp_wrt_Va / MAV.mass
    a_V2 = diffTp_wrt_delta_t / MAV.mass
    a_V3 = MAV.gravity * np.cos(theta_trim - alpha_trim)

    return Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_V1, a_V2, a_V3


def compute_ss_model(mav, trim_state, trim_input):
    x_euler = euler_state(trim_state)
    A = df_dx(mav, x_euler, trim_input)
    B = df_du(mav, x_euler, trim_input)
    # extract longitudinal states (u, w, q, theta, pd) and change pd to h
    E_1 = np.zeros((5,12),dtype=float)
    np.put(E_1, [3,17,34,43,50], [1.,1.,1.,1.,-1.])
    E_2 = np.array([[0,1,0,0],[1,0,0,0]],dtype=float)
    A_lon = E_1 @ A @ E_1.T
    B_lon = E_1 @ B @ E_2.T
    # extract lateral states (v, p, r, phi, psi)
    E_3 = np.zeros((5,12),dtype=float)
    np.put(E_3, [4,21,35,42,56], [1.,1.,1.,1.,1.])
    E_4 = np.array([[0,0,1,0],[0,0,0,1]],dtype=float)
    A_lat = E_3 @ A @ E_3.T
    B_lat = E_3 @ B @ E_4.T
    return A_lon, B_lon, A_lat, B_lat

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
    phi,theta,psi = Quaternion2Euler(x_quat[6:10])
    x_euler = np.array([[x_quat[0,0]],[x_quat[1,0]],[x_quat[2,0]],[x_quat[3,0]],[x_quat[4,0]],[x_quat[5,0]],[phi],[theta],[psi],[x_quat[10,0]],[x_quat[11,0]],[x_quat[12,0]]])
    return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions
    quat = Euler2Quaternion(x_euler[6,0],x_euler[7,0],x_euler[8,0])
    x_quat = np.array([[x_euler[0,0]],[x_euler[1,0]],[x_euler[2,0]],[x_euler[3,0]],[x_euler[4,0]],[x_euler[5,0]],[quat[0,0]],[quat[1,0]],[quat[2,0]],[quat[3,0]],[x_euler[9,0]],[x_euler[10,0]],[x_euler[11,0]]])
    return x_quat

def f_euler(mav, x_euler, delta):
    # return 12x1 dynamics (as if state were Euler state)
    # compute f at euler_state
    x_quat = quaternion_state(x_euler)
    mav._state = x_quat
    mav.update(delta,np.zeros((6,1)))

    dTHETA_dq = np.zeros((3,4))
    phi, theta, psi = Quaternion2Euler(x_quat)
    eps = 0.0001
    for i in range(4):
        x_quat_eps = np.copy(x_quat)
        x_quat_eps[i][0] += eps
        phi_eps, theta_eps, psi_eps = Quaternion2Euler(x_quat_eps)
        d_euler = np.array([[phi-phi_eps],[theta-theta_eps],[psi-psi_eps]])
        df_dq = (d_euler) / eps
        dTHETA_dq[:,i] = df_dq[:,0]
    dTe_dxq = np.zeros((12,13))
    dTe_dxq[0:3,0:3] = np.eye(3)
    dTe_dxq[3:6,3:6] = np.eye(3)
    dTe_dxq[6:9,6:10] = dTHETA_dq
    dTe_dxq[9:12,10:13] = np.eye(3)

    f_euler_ = dTe_dxq @ mav._state

    return f_euler_

def df_dx(mav, x_euler, delta):
    # take partial of f_euler with respect to x_euler
    eps = 0.001
    m = 12
    n = 12
    A = np.zeros((m,n))
    f_at_x = f_euler(mav, x_euler, delta)
    for i in range(n):
        x_eps = np.copy(x_euler)
        x_eps[i][0] += eps
        f_at_x_eps = f_euler(mav, x_eps, delta)
        df_dxi = (f_at_x_eps - f_at_x) / eps
        A[:,i] = df_dxi[:,0]
    return A


def df_du(mav, x_euler, delta):
    # take partial of f_euler with respect to input
    eps = 0.00001
    m = 12
    n = 4
    B = np.zeros((m,n))
    f_at_u = f_euler(mav, x_euler, delta)
    for i in range(n):
        u_eps = delta
        if i == 0:
            u_eps.aileron += eps
        elif i == 1:
            u_eps.elevator += eps
        elif i == 2:
            u_eps.rudder += eps
        elif i == 3:
            u_eps.throttle += eps
        f_at_u_eps = f_euler(mav, x_euler, u_eps)
        df_dui = (f_at_u_eps - f_at_u) / eps
        B[:,i] = df_dui[:,0]
    
    return B


def dT_dVa(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to Va
    eps = 0.0001
    mav._Va = Va + eps
    T_eps, Q_eps = mav._motor_thrust_torque(delta_t)
    mav._Va = Va 
    T, Q = mav._motor_thrust_torque(delta_t)
    return (T_eps - T) / eps

def dT_ddelta_t(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to delta_t
    mav._Va = Va
    eps = 0.0001
    T_eps, Q_eps = mav._motor_thrust_torque(delta_t+eps)
    T, Q = mav._motor_thrust_torque(delta_t)
    return (T_eps - T) / eps