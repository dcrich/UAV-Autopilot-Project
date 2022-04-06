# rrt dubins path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/16/2019 - RWB
from tracemalloc import start
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from chap11.draw_waypoints import DrawWaypoints
from chap12.draw_map import DrawMap
from chap11.dubins_parameters import DubinsParameters
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class RRTDubins:
    def __init__(self):
        self.segment_length = 300  # standard length of path segments
        self.plot_window = []
        self.plot_app = []
        self.dubins_path = DubinsParameters()

    def update(self, start_pose, end_pose, Va, world_map, radius):
        endReachedFlag = False
        #generate tree
        tree = MsgWaypoints()
        tree.type = 'dubins'
        # add the start pose to the tree
        course = np.radians(45)
        tree.add(ned=start_pose[0:3,:], airspeed=Va,
            course=start_pose[3,:], cost=0.0, parent=-1, connect_to_goal=0)
        # check to see if start_pose connects directly to end_pose
        distanceToEnd = distance(start_pose[0:3,:],end_pose[0:3,:])
        if distanceToEnd <= self.segment_length:
                # create leaf for end_pose
                cost = distanceToEnd        
                tree.add(end_pose[0:3,:], Va, end_pose[3,:], distanceToEnd, 0, 1)
                endReachedFlag = True
        while not endReachedFlag:#end node not connected to graph
            endReachedFlag = self.extend_tree(tree, end_pose, Va, world_map,radius)
            if np.size(tree.parent,0) > 1500:
                endReachedFlag == True
        # find path with minimum cost to end_node
        waypoints_not_smooth = find_minimum_path(tree, end_pose)
        print('smoothing')
        waypoints = waypoints_not_smooth
        # waypoints = self.smooth_path(waypoints_not_smooth, world_map,radius)
        # self.plot_map(world_map, tree, waypoints, waypoints, radius)
        
        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map, radius):
        # extend tree by randomly selecting pose and extending tree toward that pose
        p = random_pose(world_map,end_pose[2,:])
        vstarIndex = find_closest_configuration(p, tree) #find closest point in 'tree' to 'p'
        vstar = tree.ned[:,vstarIndex].reshape(3,1)
        unitDirection = (p[0:3,:]-vstar)/np.linalg.norm(p[0:3,:]-vstar)
        vplus = vstar + self.segment_length * unitDirection #plan_path(vstar, p, self.segment_length) # find point length 'D' from 'vstar' toward 'p'
        # vplus[2] = end_pose.item(2)
        # check if path from 'vstar' to 'vplus' is valid,
        collided = self.collision(vstar, tree.course[vstarIndex], vplus, p[3,0], world_map,radius)
        # collided = False
        if not collided:
            # calculate orientation
            course = p[3,:]#np.arctan2(unitDirection.item(1), unitDirection.item(0))
            # update cost
            cost = self.segment_length + tree.cost[vstarIndex] 
            distanceToEnd = distance(vplus,end_pose[0:3,:])
            connectToGoal = 0
            # add vplus as leaf to 'tree'
            tree.add(vplus, Va, course, cost, vstarIndex, connectToGoal) 
            if distanceToEnd <= self.segment_length:
                collided = self.collision(vplus,p[3,0], end_pose[0:3,:],end_pose[3,0], world_map,radius)
                if not collided:
                    # create leaf for end_pose
                    treelength = np.size(tree.ned,1)
                    cost = cost + distanceToEnd
                    connectToGoal = 1
                    tree.add(end_pose[0:3,:], Va, end_pose[3,:], cost, treelength-1, connectToGoal)                
        else:
            connectToGoal = 0
            
        if connectToGoal: #check if new node connected to end node
            flag = True
        else:
            flag = False
        return flag


    def plot_map(self, world_map, tree, waypoints, smoothed_waypoints, radius):
        scale = 4000
        # initialize Qt gui application and window
        self.plot_app = pg.QtGui.QApplication([])  # initialize QT
        self.plot_window = gl.GLViewWidget()  # initialize the view object
        self.plot_window.setWindowTitle('World Viewer')
        self.plot_window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(scale/20, scale/20, scale/20) # set the size of the grid (distance between each line)
        self.plot_window.addItem(grid) # add grid to viewer
        self.plot_window.setCameraPosition(distance=scale, elevation=50, azimuth=-90)
        self.plot_window.setBackgroundColor('k')  # set background color to black
        self.plot_window.show()  # display configured window
        self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        self.draw_tree(tree, radius, green)
        # draw things to the screen
        self.plot_app.processEvents()

    def draw_tree(self, tree, radius, color):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        Del = 0.05
        for i in range(1, tree.num_waypoints):
            parent = int(tree.parent.item(i))
            # self.dubins_path.update(column(tree.ned, parent), tree.course[parent],
            #                         column(tree.ned, i), tree.course[i], radius)
            points = self.points_along_path(column(tree.ned, parent), tree.course[parent],
                                    column(tree.ned, i), tree.course[i], radius)
            points = points @ R.T
            tree_color = np.tile(color, (points.shape[0], 1))
            tree_plot_object = gl.GLLinePlotItem(pos=points,
                                                color=tree_color,
                                                width=2,
                                                antialias=True,
                                                mode='line_strip')
            self.plot_window.addItem(tree_plot_object)

    def points_along_path(self, start_pose, chis, end_pose, chie, radius):
        Del = 0.05
        if distance(end_pose,start_pose) < 2*radius:
            N = int(np.linalg.norm(end_pose-start_pose) * 2)
            pointsN = np.linspace(start_pose.item(0),end_pose.item(0),N)
            pointsE = np.linspace(start_pose.item(1),end_pose.item(1),N)
            points = start_pose.item(2) * np.ones((3,N))
            points[0,:] = pointsN
            points[1,:] = pointsE
        else:
            self.dubins_path.update(start_pose,chis,end_pose,chie,radius)
            # points along start circle
            th1 = np.arctan2(self.dubins_path.p_s.item(1) - self.dubins_path.center_s.item(1),
                                self.dubins_path.p_s.item(0) - self.dubins_path.center_s.item(0))
            th1 = mod(th1)
            th2 = np.arctan2(self.dubins_path.r1.item(1) - self.dubins_path.center_s.item(1),
                                self.dubins_path.r1.item(0) - self.dubins_path.center_s.item(0))
            th2 = mod(th2)
            th = th1
            theta_list = [th]
            if self.dubins_path.dir_s > 0:
                if th1 >= th2:
                    while th < th2 + 2*np.pi:
                        th += Del
                        theta_list.append(th)
                else:
                    while th < th2:
                        th += Del
                        theta_list.append(th)
            else:
                if th1 <= th2:
                    while th > th2 - 2*np.pi:
                        th -= Del
                        theta_list.append(th)
                else:
                    while th > th2:
                        th -= Del
                        theta_list.append(th)

            points = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(theta_list[0]),
                                self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(theta_list[0]),
                                self.dubins_path.center_s.item(2)]])
            for angle in theta_list:
                new_point = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(angle),
                                        self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(angle),
                                        self.dubins_path.center_s.item(2)]])
                points = np.concatenate((points, new_point), axis=0)

            # points along straight line
            sig = 0
            while sig <= 1:
                new_point = np.array([[(1 - sig) * self.dubins_path.r1.item(0) + sig * self.dubins_path.r2.item(0),
                                        (1 - sig) * self.dubins_path.r1.item(1) + sig * self.dubins_path.r2.item(1),
                                        (1 - sig) * self.dubins_path.r1.item(2) + sig * self.dubins_path.r2.item(2)]])
                points = np.concatenate((points, new_point), axis=0)
                sig += Del

            # points along end circle
            th2 = np.arctan2(self.dubins_path.p_e.item(1) - self.dubins_path.center_e.item(1),
                                self.dubins_path.p_e.item(0) - self.dubins_path.center_e.item(0))
            th2 = mod(th2)
            th1 = np.arctan2(self.dubins_path.r2.item(1) - self.dubins_path.center_e.item(1),
                                self.dubins_path.r2.item(0) - self.dubins_path.center_e.item(0))
            th1 = mod(th1)
            th = th1
            theta_list = [th]
            if self.dubins_path.dir_e > 0:
                if th1 >= th2:
                    while th < th2 + 2 * np.pi:
                        th += Del
                        theta_list.append(th)
                else:
                    while th < th2:
                        th += Del
                        theta_list.append(th)
            else:
                if th1 <= th2:
                    while th > th2 - 2 * np.pi:
                        th -= Del
                        theta_list.append(th)
                else:
                    while th > th2:
                        th -= Del
                        theta_list.append(th)
            for angle in theta_list:
                new_point = np.array([[self.dubins_path.center_e.item(0) + self.dubins_path.radius * np.cos(angle),
                                        self.dubins_path.center_e.item(1) + self.dubins_path.radius * np.sin(angle),
                                        self.dubins_path.center_e.item(2)]])
                points = np.concatenate((points, new_point), axis=0)

            R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
            points = points @ R.T
        return points

    def collision(self, start_pose, chis, end_pose, chie, world_map, radius): 
        safetyMargin = 70 # needs to be less than 100
        collision_flag = False
        # check to see of path from start_pose to end_pose colliding with map
        pathPoints = self.points_along_path(start_pose, chis, end_pose, chie, radius)

        # see if any buildings within a bounding box
        bufferLength = safetyMargin + world_map.building_width*0.5
        NE = np.array([np.max(pathPoints[0,:]) + (bufferLength) , np.max(pathPoints[1,:]) + (bufferLength)]) #<
        SW = np.array([np.min(pathPoints[0,:]) - (bufferLength) , np.min(pathPoints[1,:]) - (bufferLength)]) #>
        bNE = world_map.building_north_east.copy()
        temp1 = bNE<NE
        temp2 = bNE>SW
        temp3 = np.concatenate((temp1, temp2),axis=1)
        temp33 = np.all(temp3,axis=1)
        nearbuildings = np.argwhere(temp33)

        heightmap = world_map.building_height.flatten()
        if nearbuildings.size > 0: # check if any buildings were found in bounding box
            for point in pathPoints: # check each point
                point =point[0:2].reshape(1,2)
                pn = point.item(0)
                pe = point.item(1)
                for bn_beInd in nearbuildings: # check each building in bounding box
                    bn_be = world_map.building_north_east[bn_beInd,:]
                    bn = bn_be.item(0)
                    be = bn_be.item(1)
                    bd = height_above_ground(heightmap, bn_beInd) # check building height
                    if bd >= start_pose.item(2)-safetyMargin: # if building needs to be avoided, check if collision possible
                        # check if point is within the footprint of the building+ a safety margin
                        checkNE = point <= (bn_be + bufferLength)
                        checkSW = point >= (bn_be - bufferLength)
                        if np.all(checkNE) and np.all(checkSW):
                            collision_flag = True
                    if collision_flag:
                        break
                if collision_flag:
                    break
        else: # if no buildings in bounding box, continue without collisions
            collision_flag = False
        # world_map.city_width
        # world_map.num_city_blocks
        # world_map.building_north
        # world_map.building_east
        # find buildings in the search field
        # check if bounding box above all buildings
        # check distance from each building
        # if collided:
        #     collision_flag = True
        # else:
        #     collision_flag = False
        return collision_flag

    def smooth_path(self, waypoints, world_map,radius):
        # smooth the waypoint path
        smooth = [0]  # add the first waypoint
        # construct smooth waypoint path
        smooth_waypoints = MsgWaypoints()
        p=0
        smooth_waypoints.add(ned=waypoints.ned[:,p].reshape(3,1), airspeed=waypoints.airspeed[p],
                course=waypoints.course[p], cost=waypoints.cost[p], parent=waypoints.parent[p], connect_to_goal=waypoints.connect_to_goal[p])
        N = waypoints.num_waypoints
        i = 0
        j = 1
        while j<N-1:
            ws = smooth_waypoints.ned[:,i]
            cs = smooth_waypoints.course[i]
            wp = waypoints.ned[:,j+1]
            cp = waypoints.course[i]
            collided = self.collision(ws,cs,wp,cp,world_map,radius)
            if collided:
                w = waypoints.ned[:,j]
                distance =np.linalg.norm(w - ws)
                unitDirection = (w - ws)/distance
                coursew = np.arctan2(unitDirection.item(1), unitDirection.item(0))
                costw= smooth_waypoints.cost[i] + distance
                parentw=i
                smooth_waypoints.add(ned=waypoints.ned[:,j].reshape(3,1), airspeed=waypoints.airspeed[j],
                course=coursew, cost=costw, parent=parentw, connect_to_goal=waypoints.connect_to_goal[j])
                i+=1
            j+=1
        ws = smooth_waypoints.ned[:,smooth_waypoints.num_waypoints-1]
        w = waypoints.ned[:,N-1]
        distance =np.linalg.norm(w - ws)
        unitDirection = (w - ws)/distance
        coursew = np.arctan2(unitDirection.item(1), unitDirection.item(0))
        costw= smooth_waypoints.cost[i] + distance
        parentw=i
        smooth_waypoints.add(ned=w.reshape(3,1), airspeed=waypoints.airspeed[N-1],
                course=coursew, cost=costw, parent=parentw, connect_to_goal=1)
        
        return smooth_waypoints


def find_minimum_path(tree, end_pose):
    # find the lowest cost path to the end node
    # find nodes that connect to end_node
    connecting_nodes = np.argwhere(tree.connect_to_goal == 1)
    connecting_node_costs = tree.cost[connecting_nodes]
    # removed bc only finding one path
    # # find minimum cost last node
    # idx = np.argmin(connecting_node_costs)
    # bestNode = connecting_nodes[idx]
    # # construct lowest cost path order
    # creatingpath = True
    # path = np.array([int(bestNode)])
    # currentNode = bestNode
    creatingpath = True
    currentNode = connecting_nodes
    path = np.array([int(currentNode)])
    while creatingpath:
        parentNode = tree.parent[int(currentNode.item(0))]
        if parentNode == -1:
            creatingpath = False
        else:
            path = np.append(path, int(parentNode))
            currentNode = parentNode
    path = np.flip(path)

    # construct waypoint path
    waypoints = MsgWaypoints()
    i = 0
    for p in path:
        if i == 0:
            # waypoints.add(ned=np.array([-20,-20,0]).reshape(3,1), airspeed=tree.airspeed[p],
            # course=np.pi/2., cost=tree.cost[p], parent=tree.parent[p], connect_to_goal=tree.connect_to_goal[p])
            w = tree.ned[:,path[1]]
            ws = tree.ned[:,p]
            unitDirection = (w - ws)/distance(ws,w)
            course = np.arctan2(unitDirection.item(1), unitDirection.item(0))
        else:
            course = tree.course[p]
        waypoints.add(ned=tree.ned[:,p].reshape(3,1), airspeed=tree.airspeed[p],
            course=course, cost=tree.cost[p], parent=tree.parent[p], connect_to_goal=tree.connect_to_goal[p])
        i+=1
    return waypoints


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(end_pose-start_pose)
    return d


def height_above_ground(world_map, pointInd):
    # find the altitude of point above ground level
    h_agl = world_map[pointInd.item(0)]
    return h_agl 


def random_pose(world_map, pd):
    # generate a random pose
    pose = np.random.uniform(0.0,2.5*world_map.city_width,4).reshape((4,1))
    pose[2] = pd
    pose[3] = np.random.uniform(0.0,2.0*np.pi)
    return pose


def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col


def find_closest_configuration(p,tree):
    distanceTracker = np.array([])
    for i in range(np.size(tree.ned,1)):
        leaf = tree.ned[:,i]
        d = distance(leaf,p)
        distanceTracker = np.append(distanceTracker,[d])
    closestLeafIndex = np.argmin(distanceTracker)
    return closestLeafIndex

def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])