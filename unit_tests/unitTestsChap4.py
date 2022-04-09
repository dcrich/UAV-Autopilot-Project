"""
Unit tests for Chap 4.
Checks:
- Thrust and Torque from throttle command
- Forces and Moments from aileron, elevator, and rudder commands
- Wind effect on Va, alpha, and beta, from each direction

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
import pytest
@pytest.fixture # create variables that can be used in each test function
def sim():
    # initialize mav
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    mav._Va = 25.0
    # initialize inputs
    delta = MsgDelta()
    delta.elevator =  0.0
    delta.aileron =   0.0
    delta.rudder =    0.0
    delta.throttle =  0.7

    # initialize wind
    wind = np.zeros((6,1))
    return mav, delta, wind



# check new mav dynamics functions, no wind
# motor torque and thrust
def test_motor_thrust_torque(sim):
    mav, delta, wind = sim
    delta.throttle = 0.9
    thrust, torque = mav._motor_thrust_torque(delta.throttle)
    assert np.isclose(thrust, trueValues.thrust)
    assert np.isclose(torque, trueValues.torque)

# forces and moments
# test elevator
def test_forces_moments_elevator(sim):
    mav, delta, wind = sim
    delta.elevator = -0.2
    forcesAndMoments = mav._forces_moments(delta)
    assert np.allclose(forcesAndMoments, trueValues.forcesAndMoments_e)

# test aileron
def test_forces_moments_aileron(sim):
    mav, delta, wind = sim
    delta.aileron =   0.1
    forcesAndMoments = mav._forces_moments(delta)
    assert np.allclose(forcesAndMoments, trueValues.forcesAndMoments_a)

# test rudder
def test_forces_moments_rudder(sim):
    mav, delta, wind = sim
    delta.rudder =    0.01
    forcesAndMoments = mav._forces_moments(delta)
    assert np.allclose(forcesAndMoments, trueValues.forcesAndMoments_r)


# check "trim" reaction to wind from each direction
# update_velocity
# north wind
def test_update_velocity_data_northwind(sim):
    mav, delta, wind = sim
    wind[0,0] = 5 # set north wind
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_n)
    assert np.isclose(mav._alpha, trueValues.alpha_n)
    assert np.isclose(mav._beta, trueValues.beta_n)

# south wind
def test_update_velocity_data_southwind(sim):
    mav, delta, wind = sim
    wind[0,0] = -5 # south wind
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_s)
    assert np.isclose(mav._alpha, trueValues.alpha_s)
    assert np.isclose(mav._beta, trueValues.beta_s)

# east wind
def test_update_velocity_data_eastwind(sim):
    mav, delta, wind = sim
    wind[1,0] = 5 # east wind
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_e)
    assert np.isclose(mav._alpha, trueValues.alpha_e)
    assert np.isclose(mav._beta, trueValues.beta_e)

# west wind
def test_update_velocity_data_westwind(sim):
    mav, delta, wind = sim
    wind[1,0] = -5 # west wind
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_w)
    assert np.isclose(mav._alpha, trueValues.alpha_w)
    assert np.isclose(mav._beta, trueValues.beta_w)

# up wind
def test_update_velocity_data_upwind(sim):
    mav, delta, wind = sim
    wind[2,0] = 5 # up wind
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_u)
    assert np.isclose(mav._alpha, trueValues.alpha_u)
    assert np.isclose(mav._beta, trueValues.beta_u)

# down wind
def test_update_velocity_data_downwind(sim):
    mav, delta, wind = sim
    wind[2,0] = -5 # down wind
    mav._update_velocity_data(wind)
    assert np.isclose(mav._Va, trueValues.Va_d)
    assert np.isclose(mav._alpha, trueValues.alpha_d)
    assert np.isclose(mav._beta, trueValues.beta_d)


