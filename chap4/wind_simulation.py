"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
import sys
sys.path.append('..')
from tools.transfer_function import transferFunction
import numpy as np


class WindSimulation:
    def __init__(self, Ts):
        # steady state wind defined in the inertial frame
        self._steady_state = np.array([[.0, 1., 0.]]).T
        # self._steady_state = np.array([[0., 0., 0.]]).T

        #   Dryden gust model parameters (section 4.4 UAV book)
        Va = 1 # must set Va to a constant value
        Lu = 200
        Lv = 200
        Lw = 50
        gust_flag = True
        if gust_flag==True:
            sigma_u = 1.06
            sigma_v = 1.06
            sigma_w = 0.7
        else:
            sigma_u = 0.0
            sigma_v = 0.0
            sigma_w = 0.0

        # Dryden transfer functions (section 4.4 UAV book)
        num_u_w = np.array([[sigma_u * np.sqrt(2 * Va / (np.pi * Lu))]])
        den_u_w = np.array([[1, Va/Lw]])
        self.u_w = transferFunction(num=num_u_w, den=den_u_w, Ts=Ts)

        tempvar = sigma_v * np.sqrt(3 * Va / (np.pi * Lv))
        num_v_w = np.array([[tempvar, tempvar * Va / (np.sqrt(3)*Lv) ]])
        den_v_w = np.array([[1, 2*Va/Lv, (Va/Lv)**2]])
        self.v_w = transferFunction(num=num_v_w, den=den_v_w, Ts=Ts)

        tempvar = sigma_w * np.sqrt(3 * Va / (np.pi * Lw))
        num_w_w = np.array([[tempvar, tempvar * Va / (np.sqrt(3)*Lw) ]])
        den_w_w = np.array([[1, 2*Va/Lw, (Va/Lw)**2]])
        self.w_w = transferFunction(num=num_w_w, den=den_w_w, Ts=Ts)
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn(1))],
                         [self.v_w.update(np.random.randn(1))],
                         [self.w_w.update(np.random.randn(1))]])
        # gust = np.array([[0.],[0.],[0.]])
        return np.concatenate(( self._steady_state, gust ))

