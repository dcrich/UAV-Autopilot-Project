"""
Unit tests for Chap 4.
Checks 

Run in terminal using commands:
'cd <file directory>'   e.g.-> 'cd /Users/danada/Coding/Flight Controls EE 674/mavsim_python'
'pytest -q unit_tests/unitTestsChap4.py'
"""

import sys
sys.path.append('/Users/danada/Coding/Flight Controls EE 674/mavsim_python') #sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap4.mav_dynamics import MavDynamics
from message_types.msg_delta import MsgDelta
import unit_tests.chap4_truth as trueValues


# check mav dynamics functions, no wind
# derivative shouldn't have changed since chap 3
# update shouldn't have changed since chap 3
# motor torque and thrust
def test_motor_thrust_torque():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    delta_t = 0.9
    thrust, torque = mav._motor_thrust_torque(delta_t)

    assert np.isclose(thrust, trueValues.thrust)
    assert np.isclose(torque, trueValues.torque)

# forces and moments
# test elevator
def test_forces_moments_elevator():
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    delta = MsgDelta()
    delta.elevator = -0.2
    delta.aileron =   0.0
    delta.rudder =    0.0
    delta.throttle =  0.7
    forcesAndMoments = mav._forces_moments(delta)
    assert np.allclose(forcesAndMoments, trueValues.forcesAndMoments_e)

# test aileron
def test_forces_moments_aileron():
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    delta = MsgDelta()
    delta.elevator =  0.0
    delta.aileron =   0.1
    delta.rudder =    0.0
    delta.throttle =  0.7
    forcesAndMoments = mav._forces_moments(delta)
    assert np.allclose(forcesAndMoments, trueValues.forcesAndMoments_a)

# test rudder
def test_forces_moments_rudder():
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    delta = MsgDelta()
    delta.elevator =  0.0
    delta.aileron =   0.0
    delta.rudder =    0.01
    delta.throttle =  0.7
    forcesAndMoments = mav._forces_moments(delta)
    assert np.allclose(forcesAndMoments, trueValues.forcesAndMoments_r)



# check "trim" reaction to wind from each direction
# update_velocity
# north wind
def test_update_velocity_data_northwind():
    wind = np.zeros((6,1))
    wind[0,0] = 5 # set north wind
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_n)
    assert np.isclose(mav._alpha, trueValues.alpha_n)
    assert np.isclose(mav._beta, trueValues.beta_n)

# south wind
def test_update_velocity_data_southwind():
    wind = np.zeros((6,1))
    wind[0,0] = -5 # south wind
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_s)
    assert np.isclose(mav._alpha, trueValues.alpha_s)
    assert np.isclose(mav._beta, trueValues.beta_s)

# east wind
def test_update_velocity_data_eastwind():
    wind = np.zeros((6,1))
    wind[1,0] = 5 # east wind
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_e)
    assert np.isclose(mav._alpha, trueValues.alpha_e)
    assert np.isclose(mav._beta, trueValues.beta_e)

# west wind
def test_update_velocity_data_westwind():
    wind = np.zeros((6,1))
    wind[1,0] = -5 # west wind
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_w)
    assert np.isclose(mav._alpha, trueValues.alpha_w)
    assert np.isclose(mav._beta, trueValues.beta_w)

# up wind
def test_update_velocity_data_upwind():
    wind = np.zeros((6,1))
    wind[2,0] = 5 # up wind
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_u)
    assert np.isclose(mav._alpha, trueValues.alpha_u)
    assert np.isclose(mav._beta, trueValues.beta_u)

# down wind
def test_update_velocity_data_downwind():
    wind = np.zeros((6,1))
    wind[2,0] = -5 # down wind
    mav = MavDynamics(SIM.ts_simulation)
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_d)
    assert np.isclose(mav._alpha, trueValues.alpha_d)
    assert np.isclose(mav._beta, trueValues.beta_d)



