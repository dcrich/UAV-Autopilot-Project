import numpy as np
from math import sin, cos
import sys

sys.path.append('..')
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap


class PathFollower:
    def __init__(self):
        self.chi_inf = np.pi/5.0 #between 0 (parallel), pi (normal) # approach angle for large distance from straight-line path
        self.k_path = .05 # <1 (smooth), >1 (abrupt)#0.05  # proportional gain for straight-line path following
        self.k_orbit = 10. # 10.0  # proportional gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        # path.line_origin     # origin of the straight path line (r), 3x1 array
        # path.line_direction  # direction of line -unit vector- (q), 3x1 array
        q = path.line_direction.copy()
        q_n = q.item(0)
        q_e = q.item(1)
        q_d = q.item(2)
        r_i = path.line_origin.copy()
        r_n = r_i.item(0)
        r_e = r_i.item(1)
        r_d = r_i.item(2)
        p_i = np.array([state.north,state.east,-state.altitude]).T #should altitude be negative ??????????????????????????????????????????????????
        p_n = p_i.item(0)
        p_e = p_i.item(1)
        p_d = p_i.item(2)

        chi_q = wrap(np.arctan2(q_e,q_n),state.chi)
        R_i_Path = np.array([[np.cos(chi_q),  np.sin(chi_q), 0],
                             [-np.sin(chi_q), np.cos(chi_q), 0],
                             [0, 0, 1]])
        # e_p = R_i_Path * (p_i - r_i)
        e_py = -np.sin(chi_q) * (p_n-r_n) + np.cos(chi_q) * (p_e-r_e)
        chi_c = chi_q - self.chi_inf * 2.0 * np.arctan(self.k_path * e_py) / np.pi

        e_p_i = p_i - r_i
        k = np.array([0,0,1])
        q_cross_k = np.cross(q.flatten(),k)
        n = q_cross_k / np.linalg.norm(q_cross_k)
        s_i = e_p_i - (e_p_i @ n)*n
        s_n = s_i.item(0)
        s_e = s_i.item(1)
        h_d = -r_d - np.sqrt(s_n**2 + s_e**2) * (q_d / np.sqrt(q_n**2 + q_e**2))

        self.autopilot_commands.airspeed_command = path.airspeed
        # course command
        self.autopilot_commands.course_command = chi_c
        # altitude command
        self.autopilot_commands.altitude_command = h_d
        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0.0

    def _follow_orbit(self, path, state):
        p_n = state.north
        p_e = state.east
        p_d = -state.altitude
        c_n = path.orbit_center.item(0)
        c_e = path.orbit_center.item(1)
        c_d = path.orbit_center.item(2)
        rho = path.orbit_radius
        h_c = -c_d
        if path.orbit_direction == 'CW': direction = 1.0
        else: direction = -1.0
        # airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed
        # distance from orbit center
        d = np.sqrt(((p_n - c_n)**2 + (p_e - c_e)**2))
        # compute wrapped version of angular position on orbit
        varphi = wrap(np.arctan2(p_e - c_e, p_n - c_n), state.chi)
        chi_o = varphi + direction * np.pi / 2.0
        # compute normalized orbit error
        orbit_error = np.abs((d-rho) / rho)
        # course command
        chi_d = chi_o + direction * np.arctan(self.k_orbit * (d - rho) / rho)
        self.autopilot_commands.course_command = chi_d
        # altitude command
        self.autopilot_commands.altitude_command = h_c
        # roll feedforward command
        if orbit_error < 0.05: #d-rho > -15 or d-rho < 15:#
            self.autopilot_commands.phi_feedforward = direction * np.arctan(state.Va**2 / (self.gravity * rho))#direction * np.arctan(state.Vg**2 / (self.gravity * rho * np.cos(state.chi - state.psi)))
        else:
            self.autopilot_commands.phi_feedforward = 0.0



