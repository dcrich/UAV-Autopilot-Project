# Unit Tests Chap 3
import sys
sys.path.append('/Users/danada/Coding/Flight Controls EE 674/mavsim_python')
import numpy as np
import parameters.simulation_parameters as SIM

from chap3.mav_dynamics import MavDynamics
from message_types.msg_delta import MsgDelta

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time


# check fx
def checkfx():
    fx = 0  # 10
    fy = 0  # 10
    fz = 0  # 10
    Mx = 0  # 0.1
    My = 0  # 0.1
    Mz = 0  # 0.1
    forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T

    # -------physical system-------------
    mav.update(forces_moments)  # propagate the MAV dynamics

    assert mav.true_state

# check fy


# check fz


# check Mx


# check My


# check Mz


