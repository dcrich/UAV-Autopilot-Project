"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import sys
import numpy as np
from scipy import stats
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors

class Observer:
    def __init__(self, ts_control, initial_state = MsgState(), initial_measurements = MsgSensors()):
        # initialized estimated state message
        self.estimated_state = initial_state
        # use alpha filters to low pass filter gyros and accels
        # alpha = Ts/(Ts + tau) where tau is the LPF time constant
        self.lpf_gyro_x = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_z)
        # use alpha filters to low pass filter absolute and differential pressure
        self.lpf_abs = AlphaFilter(alpha=0.9, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.7, y0=initial_measurements.diff_pressure)
        # ekf for phi and theta
        self.attitude_ekf = EkfAttitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = EkfPosition()


    def update(self, measurement):
        # estimates for p, q, r are low pass filter of gyro minus bias estimate -- ch 8 slide 11
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) - self.estimated_state.bx
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) - self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) - self.estimated_state.bz

        # invert sensor model to get altitude and airspeed -- ch 8 slide 5 
        self.estimated_state.altitude = self.lpf_abs.update(measurement.abs_pressure) / (CTRL.rho * CTRL.gravity)
        self.estimated_state.Va = np.sqrt( 2.0 * self.lpf_diff.update(measurement.diff_pressure) / CTRL.rho)

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state


class AlphaFilter:
    # alpha filter implements a simple low pass filter -- ch 8 slide 11
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.alpha * self.y + (1-self.alpha) * u
        return self.y


class EkfAttitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        self.Q = np.array([[1., 
                            1.]]).T #tune    
        self.Q = 10**-6 * self.Q                                                                    
        self.Q_gyro = SENSOR.gyro_sigma**2 * np.eye(4)
        self.R_accel = SENSOR.accel_sigma**2 * np.eye(3)
        self.N = 4.0  # number of prediction step per sample 
        self.xhat =  np.array([[CTRL.MAV.phi0],[CTRL.MAV.theta0]])# initial state: phi, theta
        self.P = np.eye(2,dtype=float)
        self.Ts = SIM.ts_control              
        self.gate_threshold = stats.chi2.isf(q=0.01,df=2)

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x, measurement, state):                    
        # system dynamics for propagation model: xdot = f(x, u)
        p = measurement.gyro_x
        q = measurement.gyro_y
        r = measurement.gyro_z
        phi = x.item(0)
        theta = x.item(1)
        f_ = np.array([[p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)],
                       [q * np.cos(phi) - r * np.sin(phi)]])
        return f_

    def h(self, x, measurement, state):
        # measurement model y
        p = measurement.gyro_x
        q = measurement.gyro_y
        r = measurement.gyro_z
        Va = np.sqrt( 2.0 * measurement.diff_pressure / CTRL.rho) # state.Va
        phi = x.item(0)
        theta = x.item(1)
        h_ = np.array([[q * Va * np.sin(theta) + CTRL.gravity * np.sin(theta)],
                       [r * Va * np.cos(theta) - p * Va * np.sin(theta) - CTRL.gravity * np.cos(theta)* np.sin(phi)],
                       [- q * Va * np.cos(theta) - CTRL.gravity * np.cos(theta) * np.cos(phi)]])
        return h_

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, int(self.N)):
            T_p = self.Ts / self.N
            # propagate model
            self.xhat = self.xhat + T_p * self.f(self.xhat,measurement,state)
            phi = self.xhat.item(0)
            theta = self.xhat.item(0)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # compute G matrix for gyro noise
            G = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta), 0],
                          [0, np.cos(phi), -np.sin(phi), 0]])
            # convert to discrete time models
            I = np.eye(np.size(A,0))
            A_d = I + A * T_p + A @ A.T * T_p**2
            # update P with discrete time model
            cov = G @ self.Q_gyro @ G.T + self.Q
            self.P = A_d @ self.P @ A_d.T + T_p**2 * cov

    def measurement_update(self, measurement, state):
        # measurement updates
        h = self.h(self.xhat, measurement, state)
        C = jacobian(self.h, self.xhat, measurement, state)
        y = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T
        S_inv = np.linalg.inv(self.R_accel + C @ self.P @ C.T)
        if True:#(y-h).T @ S_inv @ (y-h) < self.gate_threshold:
            L = self.P @ C.T @ S_inv
            tempvar = np.eye(2) - L @ C
            self.P = tempvar @ self.P @ tempvar.T + L @ self.R_accel @ L.T
            self.xhat = self.xhat + L @ (y-h)
            #print('updating')


class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self):
        self.Q = 10**0 * np.eye(7)
        self.Q[2,2] = 10.0**0 #Vg
        self.Q[3,3] = 10**-4 #chi
        self.Q[4,4] = 10**-3 #wn
        self.Q[5,5] = 10**-3 #we
        self.Q[6,6] = 10**-4 #psi
        self.R_gps = np.eye(4,dtype=float)
        self.R_gps[0,0] = SENSOR.gps_n_sigma**2
        self.R_gps[1,1] = SENSOR.gps_e_sigma**2
        self.R_gps[2,2] = SENSOR.gps_Vg_sigma**2
        self.R_gps[3,3] = SENSOR.gps_course_sigma**2
        self.R_pseudo = np.eye(2,dtype=float)
        self.N = 2.0  # number of prediction step per sample
        self.Ts = SIM.ts_control 
        self.xhat = np.array([[CTRL.MAV.north0],
                              [CTRL.MAV.east0],
                              [CTRL.MAV.Va0],
                              [CTRL.MAV.psi0],
                              [0.0],
                              [0.0],
                              [CTRL.MAV.psi0]])
        self.P = np.eye(7)
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        self.pseudo_threshold = stats.chi2.isf(0.5,df=2)

        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.north = self.xhat.item(0)
        state.east = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        Va = np.sqrt( 2.0 * measurement.diff_pressure / CTRL.rho)
        q = measurement.gyro_y
        r = measurement.gyro_z
        phi = state.phi
        theta = state.theta
        psidot = q * np.sin(phi) / np.cos(theta) + r * np.cos(phi) * np.cos(theta)
        Vgdot = ((Va * np.cos(psi) + wn) * (-Va * psidot * np.sin(psi)) + (Va * np.sin(psi) + we) * (Va * psidot * np.cos(psi)))/Vg
        f_ = np.array([[Vg * np.cos(chi)],
                       [Vg * np.sin(chi)],
                       [Vgdot],
                       [CTRL.gravity / Vg * np.tan(phi)],
                       [0],
                       [0],
                       [psidot]])# CTRL.gravity / Vg * np.tan(phi)* np.cos(chi - psi)],
        return f_

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        Va = np.sqrt( 2.0 * measurement.diff_pressure / CTRL.rho)
        h_ = np.array([[pn],
                       [pe],
                       [Vg],
                       [chi]])
                    #    [Va * np.cos(psi) + wn - Vg * np.cos(chi)],
                    #    [Va * np.sin(psi) + we - Vg * np.sin(chi)]])
        return h_

    def h_pseudo(self, x, measurement, state):
        # measurement model for wind triangale pseudo measurement
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        Va = np.sqrt( 2.0 * measurement.diff_pressure / CTRL.rho)
        h_ = np.array([[Va * np.cos(psi) + wn - Vg * np.cos(chi)],
                       [Va * np.sin(psi) + we - Vg * np.sin(chi)]])
        return h_

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, int(self.N)):
            T_p = self.Ts / self.N
            # propagate model
            self.xhat = self.xhat + T_p * self.f(self.xhat,measurement,state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # convert to discrete time models
            I = np.eye(np.size(A,0))
            A_d = I + A * T_p + A @ A.T * T_p**2
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + T_p**2 * self.Q
            if np.max(self.P)>10**5:
                stophere = 1

    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudu measurement
        h = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0.0, 0.0]]).T #np.array([[state.wn, state.we]]).T #
        S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            L = self.P @ C.T @ S_inv
            tempvar = np.eye(7) - L @ C
            self.P = tempvar @ self.P @ tempvar.T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y-h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
                        #    0.0,
                        #    0.0]]).T
            S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                L = self.P @ C.T @ S_inv 
                tempvar = np.eye(7) - L @ C
                self.P = tempvar @ self.P @ tempvar.T + L @ self.R_gps @ L.T
                self.xhat = self.xhat + L @ (y-h)

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course


def jacobian(fun, x, measurement, state):
    # compute jacobian of fun with respect to x
    f = fun(x, measurement, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J
