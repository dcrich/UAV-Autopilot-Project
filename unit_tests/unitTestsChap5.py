"""
Unit tests for Chap 5.
Checks:
- Trim
- f_euler
- df_dx
- df_du
- SS model
- TF model

Run in terminal using commands:
'cd <file directory>'   e.g.-> 'cd /Users/danada/Coding/Flight Controls EE 674/mavsim_python'
'pytest -q unit_tests/unitTestsChap4.py'
"""

import sys
sys.path.append('/Users/danada/Coding/Flight Controls EE 674/mavsim_python') #sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.mav_viewer import MavViewer
from chap3.data_viewer import DataViewer
from chap4.mav_dynamics import MavDynamics
from chap4.wind_simulation import WindSimulation
from chap5.trim import compute_trim
import chap5.compute_models as cm
from tools.signals import Signals
import unit_tests.chap5_truth as trueValues
from message_types.msg_delta import MsgDelta


# test trim
def test_trim():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    gamma = 0.*np.pi/180.
    trimState, trimInput = compute_trim(mav, mav._Va, gamma)
    print(trimState)
    print(trimInput)
    assert np.allclose(trimState, trueValues.trimState)
    assert np.isclose(trimInput.aileron, trueValues.trimInputA)
    assert np.isclose(trimInput.elevator, trueValues.trimInputE)
    assert np.isclose(trimInput.rudder, trueValues.trimInputR)
    assert np.isclose(trimInput.throttle, trueValues.trimInputT)

# test f_euler
def test_f_euler():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    Va = 25.
    gamma = 0.*np.pi/180.
    trim_state, trim_input = compute_trim(mav, Va, gamma)
    mav._state = trim_state
    x_euler = np.array([[-5.95888304e-15],
                        [ 0.00000000e+00],
                        [-1.00000000e+02],
                        [ 2.49687427e+01],
                        [ 0.00000000e+00],
                        [ 1.24975516e+00],
                        [ 0.00000000e+00],
                        [ 5.00109104e-02],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00]])
    delta = MsgDelta(-0.1247780701597401,0.0018361809638628682,-0.000302608037876341,0.6767522859047431)

    f_euler_Output = cm.f_euler(mav, x_euler, delta)
    assert np.allclose(f_euler_Output,trueValues.fEuler)

# test df_dx
def test_df_dx():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    Va = 25.
    gamma = 0.*np.pi/180.
    trim_state, trim_input = compute_trim(mav, Va, gamma)
    mav._state = trim_state
    x_euler = np.array([[-5.95888304e-15],
                        [ 0.00000000e+00],
                        [-1.00000000e+02],
                        [ 2.49687427e+01],
                        [ 0.00000000e+00],
                        [ 1.24975516e+00],
                        [ 0.00000000e+00],
                        [ 5.00109104e-02],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00]])
    delta = MsgDelta(-0.1247780701597401,0.0018361809638628682,-0.000302608037876341,0.6767522859047431)
    A = cm.df_dx(mav, x_euler, delta)
    tempvar = A - trueValues.A
    assert np.allclose(A,trueValues.A,)

# test df_du
def test_df_du():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    Va = 25.
    gamma = 0.*np.pi/180.
    trim_state, trim_input = compute_trim(mav, Va, gamma)
    mav._state = trim_state
    x_euler = np.array([[-5.95888304e-15],
                        [ 0.00000000e+00],
                        [-1.00000000e+02],
                        [ 2.49687427e+01],
                        [ 0.00000000e+00],
                        [ 1.24975516e+00],
                        [ 0.00000000e+00],
                        [ 5.00109104e-02],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00],
                        [ 0.00000000e+00]])
    B = cm.df_du(mav, x_euler, trim_input)
    assert np.allclose(B, trueValues.B)
test_df_du()

# test compute SS model
def test_compute_SS():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    Va = 25.
    gamma = 0.*np.pi/180.
    trim_state, trim_input = compute_trim(mav, Va, gamma)
    mav._state = trim_state
    A_lon, B_lon, A_lat, B_lat = cm.compute_ss_model(mav, trim_state, trim_input)
    assert np.allclose(A_lon,trueValues.A_lon)
    assert np.allclose(B_lon,trueValues.B_lon)
    assert np.allclose(A_lat,trueValues.A_lat)
    assert np.allclose(B_lat,trueValues.B_lat)


# test compute TF model
def test_compute_TF():
     # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    Va = 25.
    gamma = 0.*np.pi/180.
    trim_state, trim_input = compute_trim(mav, Va, gamma)
    mav._state = trim_state
    Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3,a_V1, a_V2, a_V3 \
        = cm.compute_tf_model(mav, trim_state, trim_input)

    assert np.allclose(Va_trim, trueValues.Va_trim)
    assert np.allclose(alpha_trim, trueValues.alpha_trim)
    assert np.allclose(theta_trim, trueValues.theta_trim)
    assert np.allclose(a_phi1, trueValues.a_phi1)
    assert np.allclose(a_phi2, trueValues.a_phi2)
    assert np.allclose(a_theta1, trueValues.a_theta1)
    assert np.allclose(a_theta2, trueValues.a_theta2)
    assert np.allclose(a_theta3, trueValues.a_theta3)
    assert np.allclose(a_V1, trueValues.a_V1)
    assert np.allclose(a_V2, trueValues.a_V2)
    assert np.allclose(a_V3, trueValues.a_V3)