import numpy as np
import sys
sys.path.append('..')
from chap11.dubins_parameters import DubinsParameters
from message_types.msg_path import MsgPath


class PathManager:
    def __init__(self):
        # message sent to path follower
        self.path = MsgPath()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        self.manager_requests_waypoints = True
        self.dubins_path = DubinsParameters()

    def update(self, waypoints, radius, state):
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def initialize_pointers(self):
        if self.num_waypoints >= 3:
            self.ptr_previous = 0
            self.ptr_current = 1
            self.ptr_next = 2
        else:
            print('Error Path Manager: need at least three waypoints')

    def increment_pointers(self):
        self.ptr_previous = self.ptr_current
        self.ptr_current = self.ptr_next
        if self.ptr_next == self.num_waypoints-1:
            self.ptr_next = 0
        else:
            self.ptr_next = self.ptr_next+1

    def inHalfSpace(self, pos):
        tempvar = pos-self.halfspace_r
        inhalfspace = tempvar.T @ self.halfspace_n
        if inhalfspace >= 0: #implement code here
            return True
        else:
            return False

    def line_manager(self, waypoints, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            self.initialize_pointers()
            self.num_waypoints = waypoints.num_waypoints
        # state machine for line path
        self.construct_line(waypoints)
        if self.inHalfSpace(mav_pos):
            self.increment_pointers()
            self.path.plot_updated = False
        self.path.plot_updated = False

    def construct_line(self, waypoints):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        if self.ptr_current == 9999:#doesn't enter
            current = 9999
        else:
            current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        if self.ptr_next == 9999:#doesn't enter
            next = 9999
        else:
            next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        #update path variables
        tempvar = current - previous
        q_im1 = tempvar / np.linalg.norm(tempvar)
        tempvar = next - current
        q_i = tempvar / np.linalg.norm(tempvar)
        if np.isnan(q_i[0,0]):
            stophere = 1
        tempvar = q_im1 + q_i 
        self.halfspace_n = tempvar / np.linalg.norm(tempvar)
        self.halfspace_r = current
        self.path.line_origin = previous
        self.path.line_direction = q_im1
        

    def fillet_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            self.manager_state == 1
        # state machine for fillet path

        if self.manager_state == 1:
            self.path.type = 'line'
            self.construct_fillet_line(waypoints,radius)
            if self.inHalfSpace(mav_pos):
                # self.increment_pointers()
                self.path.plot_updated = False
                self.manager_state = 2
        elif self.manager_state == 2:
            self.path.type = 'orbit'
            self.construct_fillet_circle(waypoints,radius)
            if self.inHalfSpace(mav_pos):
                self.increment_pointers()
                # self.path.plot_updated = False
                self.manager_state = 1
        self.path.plot_updated = False

        
    def construct_fillet_line(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        if self.ptr_current == 9999:#doesn't enter
            current = 9999
        else:
            current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        if self.ptr_next == 9999:#doesn't enter
            next = 9999
        else:
            next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        #update path variables
        tempvar = current - previous
        q_im1 = tempvar / np.linalg.norm(tempvar)
        tempvar = next - current
        q_i = tempvar / np.linalg.norm(tempvar)
        vartheta = np.arccos(-q_im1.T @ q_i)
        self.halfspace_n = q_im1
        self.halfspace_r = current - (radius / np.tan (vartheta/2.0)) * q_im1
        self.path.line_origin = previous
        self.path.line_direction = q_im1
       

    def construct_fillet_circle(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        if self.ptr_current == 9999:#doesn't enter
            current = 9999
        else:
            current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        if self.ptr_next == 9999:#doesn't enter
            next = 9999
        else:
            next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        #update path variables
        tempvar = current - previous
        q_im1 = tempvar / np.linalg.norm(tempvar)
        tempvar = next - current
        q_i = tempvar / np.linalg.norm(tempvar)
        vartheta = np.arccos(-q_im1.T @ q_i)
        self.halfspace_n = q_i
        self.halfspace_r = current + (radius / np.tan (vartheta/2.0)) * q_i
        tempvar = q_im1 - q_i 
        tempvar2 = tempvar / np.linalg.norm(tempvar)
        self.path.orbit_center = current - (radius / np.sin(vartheta/2.0)) * tempvar2
        self.path.orbit_radius = radius
        self.path.orbit_direction = np.sign(q_im1.item(0) * q_i.item(1) - q_im1.item(1) * q_i.item(0))

    def dubins_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        # state machine for dubins path

    def construct_dubins_circle_start(self, waypoints, dubins_path):
        #update path variables

    def construct_dubins_line(self, waypoints, dubins_path):
        #update path variables

    def construct_dubins_circle_end(self, waypoints, dubins_path):
        #update path variables

