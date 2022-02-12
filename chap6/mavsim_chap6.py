"""
mavsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/5/2019 - RWB
        2/24/2020 - RWB
"""
import sys
sys.path.append('/Users/danada/Coding/Flight Controls EE 674/mavsim_python')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.mav_viewer import MavViewer
from chap3.data_viewer import DataViewer
from chap4.mav_dynamics import MavDynamics
from chap4.wind_simulation import WindSimulation
from chap6.autopilot import Autopilot
#from chap6.autopilot_tecs import Autopilot
from tools.signals import Signals

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap6_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)

# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
Va_command = Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     frequency=0.01)
altitude_command = Signals(dc_offset=50.0,
                           amplitude=20.0,
                           start_time=0.0,
                           frequency=0.02)
course_command = Signals(dc_offset=np.radians(180),
                         amplitude=np.radians(15),
                         start_time=20.0,
                         frequency=0.07)

roll_command = Signals(dc_offset=np.radians(0),
                         amplitude=np.radians(15),
                         start_time=3.0,
                         frequency=0.1)

# initialize the simulation time
sim_time = SIM.start_time
commands.airspeed_command = 25.#Va_command.square(sim_time)
commands.course_command = 0.0#course_command.square(sim_time)
commands.altitude_command = 1.0
# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------autopilot commands-------------
    
    # if sim_time > 5 and sim_time <25:
    #     commands.airspeed_command = 30.0
    #     commands.course_command = 0.0
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 25 and sim_time < 50:
    #     commands.airspeed_command = 25.0
    #     commands.course_command = np.pi - .001
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 50 and sim_time < 55:
    #     commands.airspeed_command = 25.0
    #     commands.course_command = np.pi - np.pi/4.0
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 55 and sim_time < 60:
    #     commands.airspeed_command = 25.0
    #     commands.course_command = np.pi/4.0 - np.pi
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 60 and sim_time < 70:
    #     commands.airspeed_command = 25.0
    #     commands.course_command = np.pi - np.pi/4.0
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 70 and sim_time < 80:
    #     commands.airspeed_command = 25.0
    #     commands.course_command =  np.pi/4.0 - np.pi
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 80 and sim_time < 90:
    #     commands.airspeed_command = 25.0
    #     commands.course_command = np.pi
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 90 and sim_time < 110:
    #     commands.airspeed_command = 30.0
    #     commands.course_command = 2.0*np.pi
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 110 and sim_time < 125:
    #     commands.airspeed_command = 40.0
    #     commands.course_command = 0.0
    #     commands.altitude_command = 50.0
   
    # elif sim_time > 125 and sim_time < 140:
    #     commands.airspeed_command = 20.0
    #     commands.course_command = 0.0
    #     commands.altitude_command = 150.0
    
    # elif sim_time > 140 and sim_time < 150:
    #     commands.airspeed_command = 30.0
    #     commands.course_command = 0.0
    #     commands.altitude_command = 100.0
    
    # elif sim_time > 150 :
    #     commands.airspeed_command = 25.0
    #     commands.course_command = 0.0
    #     commands.altitude_command = 10.0

    # # TEST FLIGHT 2
    if sim_time > 5 and sim_time <25: # check climb
        commands.airspeed_command = 25.0
        commands.course_command = 0.0
        commands.altitude_command = 100.0
    elif sim_time > 25 and sim_time < 50: # check course
        commands.airspeed_command = 25.0
        commands.course_command = np.pi - .001
        commands.altitude_command = 100.0
    elif sim_time > 50 and sim_time < 60: # check speed
        commands.airspeed_command = 35.0
        commands.course_command = np.pi
        commands.altitude_command = 100.0
    elif sim_time > 60 and sim_time < 75: # check course and 180 degree behavior
        commands.airspeed_command = 35.0
        commands.course_command = np.pi + np.pi/6.
        commands.altitude_command = 100.0
    elif sim_time > 75 and sim_time < 90: # check course and 180 degree behavior
        commands.airspeed_command = 35.0
        commands.course_command = np.pi - np.pi/6.
        commands.altitude_command = 100.0
    elif sim_time > 90 and sim_time < 105: # check course
        commands.airspeed_command = 35.0
        commands.course_command = np.pi/2.
        commands.altitude_command = 100.0
    elif sim_time > 105 and sim_time < 115: # check decelerate
        commands.airspeed_command = 20.0
        commands.course_command = np.pi/2.
        commands.altitude_command = 100.0
    elif sim_time > 115:                  # check descend
        commands.airspeed_command = 25.0
        commands.course_command = np.pi/2.
        commands.altitude_command = 1.0
    
    # commands.phi_feedforward = 0.0#roll_command.square(sim_time)
    # if sim_time > 1:
    #     commands.phi_feedforward = 0.1
    # if sim_time > 3:
    #     commands.phi_feedforward = -0.1
    # if sim_time > 5:
    #     commands.phi_feedforward = 0.0

    # -------autopilot-------------
    estimated_state = mav.true_state  # uses true states in the control
    delta, commanded_state = autopilot.update(commands, estimated_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    data_view.update(mav.true_state,  # true states
                     estimated_state,  # estimated states
                     commanded_state,  # commanded states
                     delta,  # input to aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




