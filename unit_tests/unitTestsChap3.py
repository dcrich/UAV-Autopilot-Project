"""
Unit tests for Chap 3.
Checks the response to each force and moment when negative, positive, and zero

Run in terminal using commands:
'cd <file directory>'   e.g.-> 'cd /Users/danada/Coding/Flight Controls EE 674/mavsim_python'
'pytest -q unit_tests/unitTestsChap4.py'
"""

import sys
sys.path.append('/Users/danada/Coding/Flight Controls EE 674/mavsim_python') #sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap3.mav_dynamics import MavDynamics
import unit_tests.chap3_truth as trueValues
# import chap3_truth as trueValues


# check fx
def test_derivative_fx():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[0,0] = 10.0 # set fx
    # get derivative results
    xdotA = mav._derivatives(mav._state, forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(xdotA, trueValues.x_dot_fx)

def test_update_fx():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[0,0] = 10.0 #set fx
    # get update results
    mav.update(forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(mav._state, trueValues.state_fx)


# check fy
def test_derivative_fy():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[1,0] = 10.0 # set fy
    # get derivative results
    xdotA = mav._derivatives(mav._state, forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(xdotA, trueValues.x_dot_fy)

def test_update_fy():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[1,0] = 10.0 #set fy
    # get update results
    mav.update(forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(mav._state, trueValues.state_fy)


# check fz
def test_derivative_fz():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[2,0] = 10.0 # set fz
    # get derivative results
    xdotA = mav._derivatives(mav._state, forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(xdotA, trueValues.x_dot_fz)

def test_update_fz():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[2,0] = 10.0 #set fz
    # get update results
    mav.update(forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(mav._state, trueValues.state_fz)

# check Mx
def test_derivative_Mx():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[3,0] = 0.1 # set Mx
    # get derivative results
    xdotA = mav._derivatives(mav._state, forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(xdotA, trueValues.x_dot_Mx)

def test_update_Mx():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[3,0] = 0.1 #set Mx
    # get update results
    mav.update(forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(mav._state, trueValues.state_Mx)


# check My
def test_derivative_My():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[4,0] = 0.1 # set My
    # get derivative results
    xdotA = mav._derivatives(mav._state, forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(xdotA, trueValues.x_dot_My)

def test_update_My():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[4,0] = 0.1 #set My
    # get update results
    mav.update(forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(mav._state, trueValues.state_My)

# check Mz
def test_derivative_Mz():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[5,0] = 0.1 # set Mz
    # get derivative results
    xdotA = mav._derivatives(mav._state, forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(xdotA, trueValues.x_dot_Mz)

def test_update_Mz():
    # initialize elements of the architecture
    mav = MavDynamics(SIM.ts_simulation)
    # set state
    mav._state = np.zeros((13,1),dtype=float)
    mav._state[3,0] = 25.0 # set u0
    mav._state[6,0] = 1.0 # set e0
    # set forces and moments
    forces_moments = np.zeros((6,1),dtype=float) # fx, fy, fz, Mx, My, Mz
    forces_moments[5,0] = 0.1 #set Mz
    # get update results
    mav.update(forces_moments)  # propagate the MAV dynamics
    #compare against true values
    assert np.allclose(mav._state, trueValues.state_Mz)
test_update_Mz()
# test_derivative_fx()
# test_update_fx()
# test_derivative_fy()
# test_update_fy()
# test_derivative_fz()
# test_update_fz()
# test_derivative_Mx()
# test_update_Mx()
# test_derivative_My()
# test_update_My()
# test_derivative_Mz()
# test_update_Mz()