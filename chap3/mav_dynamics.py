"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:  
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import MsgState
import parameters.aerosonde_parameters as MAV
from tools.rotations import Quaternion2Euler, Quaternion2Rotation


class MavDynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
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
        self.true_state = MsgState()

    ###################################
    # public functions
    def update(self, forces_moments):
        '''
            Integrate the differential equations defining dynamics. 
            Inputs are the forces and moments on the aircraft.
            Ts is the time step between function calls.
        '''

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
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
        print('state')
        print(self._state)
        # update the message class for the true state
        self._update_true_state()

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
        print('xdot')
        print(x_dot)
        return x_dot

    def _update_true_state(self):
        # update the true state message:
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.Va = np.sqrt(self._state.item(3)**2 + self._state.item(4)**2 + self._state.item(5)**2)
        self.true_state.Vg = np.sqrt(self._state.item(3)**2 + self._state.item(4)**2 + self._state.item(5)**2)
        
