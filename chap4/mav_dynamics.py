"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
part of mavPySim 
    - Beard & McLain, PUP, 2012
    - Update history:  
        12/20/2018 - RWB
"""
import sys
# from typing_extensions import Self
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import MsgState

import parameters.aerosonde_parameters as MAV
from tools.rotations import Quaternion2Rotation, Quaternion2Euler


class MavDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.north0],  # (0)
                               [MAV.east0],   # (1)
                               [MAV.down0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0]])   # (12)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        self._update_velocity_data()
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.true_state = MsgState()

    ###################################
    # public functions
    def update(self, delta, wind):
        """
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        """
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

        # update the message class for the true state
        self._update_true_state()

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = state.item(0)
        # east = state.item(1)
        # down = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        phi, theta, psi = Quaternion2Euler(np.array([[e0],[e1],[e2],[e3]]))
        cphi = np.cos(phi)
        ctheta = np.cos(theta)
        cpsi = np.cos(psi)
        sphi = np.sin(phi)
        stheta = np.sin(theta)
        spsi = np.sin(psi)
        # position kinematics
        # pos_dot =
        north_dot = u * ctheta * cpsi + v * (sphi * stheta * cpsi - cphi * spsi) + w * (cphi * stheta * cpsi + sphi * spsi)
        east_dot = u * ctheta * spsi + v * (sphi * stheta * spsi + cphi * cpsi) + w * (cphi * stheta * spsi - sphi * cpsi)
        down_dot = - u * stheta + v * sphi * ctheta + w * cphi * ctheta

        # position dynamics
        u_dot = r*v - q*w + fx/MAV.mass
        v_dot = p*w - r*u + fy/MAV.mass
        w_dot = q*u - p*v + fz/MAV.mass

        # rotational kinematics
        e0_dot = 0.5 * (-e1*p - e2*q - e3*r)
        e1_dot = 0.5 * (e0*p - e3*q + e2*r)
        e2_dot = 0.5 * (e3*p + e0*q - e1*r)
        e3_dot = 0.5 * (e1*q - e2*p + e0*r)

        Jy = MAV.Jy
        Gamma1 = MAV.gamma1
        Gamma2 = MAV.gamma2
        Gamma3 = MAV.gamma3
        Gamma4 = MAV.gamma4
        Gamma5 = MAV.gamma5
        Gamma6 = MAV.gamma6
        Gamma7 = MAV.gamma7
        Gamma8 = MAV.gamma8
        # rotatonal dynamics
        p_dot = Gamma1 * p * q - Gamma2 * q * r + Gamma3 * l + Gamma4 * n
        q_dot = Gamma5 * p * r - Gamma6 * (p **2 - r **2) + m / Jy
        r_dot = Gamma7 * p * q - Gamma1 * q * r + Gamma4 * l + Gamma8 * n

        # collect the derivative of the states
        x_dot = np.array([[north_dot, east_dot, down_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]

        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        phi, theta, psi = Quaternion2Euler(np.array([[e0],[e1],[e2],[e3]]))
        cphi = np.cos(phi)
        ctheta = np.cos(theta)
        cpsi = np.cos(psi)
        sphi = np.sin(phi)
        stheta = np.sin(theta)
        spsi = np.sin(psi)
        rotationVehicleToBody = np.array([[ctheta*cpsi, ctheta*spsi, -stheta],
                                 [sphi*stheta*cpsi-cphi*spsi, sphi*stheta*spsi+cphi*cpsi, sphi*ctheta],
                                 [cphi*stheta*cpsi + sphi*spsi, cphi*stheta*spsi-sphi*cpsi, cphi*ctheta]])
        # convert wind vector from world to body frame and add gust
        wind_body_frame = rotationVehicleToBody @ steady_state + gust
        # velocity vector relative to the airmass
        # v_air = 
        u = self._state[3]
        v = self._state[4]
        w = self._state[5]
        ur = u - wind_body_frame[0] #self._Va * np.cos(self._alpha) * np.cos(self._beta)
        vr = v - wind_body_frame[1] #self._Va * np.sin(self._beta)
        wr = w - wind_body_frame[2] #self._Va * np.sin(self._alpha) * np.cos(self._beta)
        # compute airspeed
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
        # compute angle of attack
        if ur == 0:
            self._alpha = np.pi/2.0
        else:
            self._alpha = np.arctan(wr/ur)
        # compute sideslip angle
        if self._Va == 0:
            self._beta = 0
        else:
            self._beta = np.arcsin(vr / self._Va)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        e_0 = self._state.item(6)
        e_x = self._state.item(7)
        e_y = self._state.item(8)
        e_z = self._state.item(9)
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)
        delta_a = delta.aileron
        delta_e = delta.elevator
        delta_r = delta.rudder
        delta_t = delta.throttle

        # compute gravitaional forces
        f_g = np.array([MAV.mass * MAV.gravity * 2 * (e_x * e_z - e_y * e_0),
                        MAV.mass * MAV.gravity * 2 * (e_y * e_z + e_x * e_0),
                        MAV.mass * MAV.gravity * (e_z**2 + e_0**2 - e_x**2 - e_y**2)])
        # f_g = np.array([- MAV.mass * MAV.gravity * np.sin(theta),
        #                 MAV.mass * MAV.gravity * np.cos(theta)*np.sin(phi),
        #                 MAV.mass * MAV.gravity * np.cos(theta)*np.cos(phi)])

        # compute Lift and Drag coefficients
        M_transRate = MAV.M
        alpha_0 = MAV.alpha0
        sigmaOfAlpha = (1 + np.exp(-M_transRate * (self._alpha - alpha_0)) + np.exp(M_transRate * (self._alpha + alpha_0))) /       \
                       ((1 + np.exp(-M_transRate * (self._alpha - alpha_0))) * (1 + np.exp(M_transRate * (self._alpha + alpha_0))))
        CL = (1 - sigmaOfAlpha) * (MAV.C_L_0 + MAV.C_L_alpha * self._alpha) + sigmaOfAlpha * (2 * np.sign(self._alpha) * np.sin(self._alpha)**2 * np.cos(self._alpha))
        CD = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * self._alpha)**2 / (np.pi * MAV.e * MAV.AR)
        # compute Lift and Drag Forces
        F_lift = 0.5 * MAV.rho * self._Va **2 * MAV.S_wing * (CL + MAV.C_L_q * 0.5 * MAV.c / self._Va * q + MAV.C_L_delta_e * delta_e)
        F_drag = 0.5 * MAV.rho * self._Va **2 * MAV.S_wing * (CD + MAV.C_D_q * 0.5 * MAV.c / self._Va * q + MAV.C_D_delta_e * delta_e)

        #compute propeller thrust and torque
        T_p, Q_p = self._motor_thrust_torque(delta_t)

        # compute longitudinal forces in body frame
        # CX = -CD *np.cos(self._alpha) + CL*np.sin(self._alpha)
        # CXq = -MAV.C_D_q * np.cos(self._alpha) + MAV.C_L_q *np.sin(self._alpha)
        # CXdeltae = -MAV.C_D_delta_e * np.cos(self._alpha) + MAV.C_L_delta_e *np.sin(self._alpha)
        # fxlong = -MAV.mass * MAV.gravity * np.sin(theta) + \
        #         T_p + \
        #         0.5 * MAV.rho * self._Va**2 * MAV.S_wing * (CX + CXq * MAV.c * q * 0.5 /self._Va) + \
        #         0.5 * MAV.rho * self._Va**2 * MAV.S_wing * (CXdeltae * delta_e)
        fx = f_g[0] + (- np.cos(self._alpha) * F_drag + np.sin(self._alpha) * F_lift) + (T_p) # seems right
        fz = f_g[2] + (- np.sin(self._alpha) * F_drag - np.cos(self._alpha) * F_lift) # provides answer that is close

        # compute logitudinal torque in body frame
        m = 0.5 * MAV.rho * self._Va **2 * MAV.S_wing * MAV.c *                                                      \
            (MAV.C_m_0 + MAV.C_m_alpha * self._alpha +                                                               \
             MAV.C_m_q * 0.5 * MAV.c / self._Va * q + MAV.C_m_delta_e * delta_e) # seems right
        
        # compute lateral forces in body frame
        fy = 0.5 * MAV.rho * self._Va **2 * MAV.S_wing *                                                             \
            (MAV.C_Y_0 + MAV.C_Y_beta * self._beta + MAV.C_Y_p * 0.5 * MAV.b / self._Va * p +                        \
             MAV.C_Y_r * 0.5 * MAV.b / self._Va * r + MAV.C_Y_delta_a * delta_a + MAV.C_Y_delta_r * delta_r)

        # compute lateral torques in body frame
        l = 0.5 * MAV.rho * self._Va **2 * MAV.S_wing * MAV.b *                                                      \
            (MAV.C_ell_0 + MAV.C_ell_beta * self._beta + MAV.C_ell_p * 0.5 * MAV.b / self._Va * p +                  \
             MAV.C_ell_r * 0.5 * MAV.b / self._Va * r + MAV.C_ell_delta_a * delta_a + MAV.C_ell_delta_r * delta_r) - \
            Q_p
        n = 0.5 * MAV.rho * self._Va **2 * MAV.S_wing * MAV.b *                                                      \
            (MAV.C_n_0 + MAV.C_n_beta * self._beta + MAV.C_n_p * 0.5 * MAV.b / self._Va * p +                        \
             MAV.C_n_r * 0.5 * MAV.b / self._Va * r + MAV.C_n_delta_a * delta_a + MAV.C_n_delta_r * delta_r)

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        # print(np.array([[fx, fy, fz, l, m, n]]).T)
        return np.array([[fx, fy, fz, l, m, n]]).T

    def _motor_thrust_torque(self, delta_t):
        # compute thrust and torque due to propeller (See addendum by McLain) # map delta t throttle command(0 to 1) into motor input voltage
        V_in = MAV.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = MAV.C_Q0 * MAV.rho * MAV.D_prop**5 / (4.0 * np.pi**2)
        b = (MAV.C_Q1 * MAV.rho * MAV.D_prop**4 * self._Va / (2.0 * np.pi)) + (MAV.KQ **2 / MAV.R_motor)
        c = (MAV.C_Q2 * MAV.rho * MAV.D_prop**3 * self._Va**2) - (MAV.KQ * V_in / MAV.R_motor) + (MAV.KQ * MAV.i0)
        # Consider only positive root
        Omega_p = (-b + np.sqrt(b**2 - 4 * a * c)) / (2.0 * a) 
        J_op = 2 * np.pi * self._Va / (Omega_p * MAV.D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = MAV.C_T2 * J_op**2 + MAV.C_T1 * J_op + MAV.C_T0 #
        C_Q = MAV.C_Q2 * J_op**2 + MAV.C_Q1 * J_op + MAV.C_Q0 #

        # add thrust and torque due to propeller
        tempvar = Omega_p**2 / (4 * np.pi**2)
        thrust_prop = MAV.rho * tempvar * MAV.D_prop**4 * C_T
        torque_prop = MAV.rho * tempvar * MAV.D_prop**5 * C_Q 
        
        return thrust_prop, torque_prop

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        pdot = Quaternion2Rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
