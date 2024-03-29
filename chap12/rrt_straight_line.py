# rrt straight line path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - Brady Moon
#         4/11/2019 - RWB
#         3/31/2020 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from chap11.draw_waypoints import DrawWaypoints
from chap12.draw_map import DrawMap
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class RRTStraightLine:
    def __init__(self):
        self.segment_length = 300 # standard length of path segments
        self.plot_window = []
        self.plot_app = []

    def update(self, start_pose, end_pose, Va, world_map, radius):
        endReachedFlag = False
        #generate tree
        tree = MsgWaypoints()
        # tree.type = 'straight_line'
        tree.type = 'fillet'
        # add the start pose to the tree
        unitDirection = (end_pose - start_pose)/np.linalg.norm(end_pose - start_pose)
        course = np.arctan2(unitDirection.item(1), unitDirection.item(0))
        tree.add(ned=start_pose, airspeed=Va,
            course=course, cost=0.0, parent=-1, connect_to_goal=0)
        # check to see if start_pose connects directly to end_pose
        distanceToEnd = distance(start_pose,end_pose)
        if distanceToEnd <= self.segment_length:
                # create leaf for end_pose
                cost = distanceToEnd        
                tree.add(end_pose, Va, course, distanceToEnd, 0, 1)
                endReachedFlag = True
        while not endReachedFlag:#end node not connected to graph
            endReachedFlag = self.extend_tree(tree, end_pose, Va, world_map)
            if np.size(tree.parent,0) > 1500:
                endReachedFlag == True
        # find path with minimum cost to end_node
        waypoints_not_smooth = find_minimum_path(tree, end_pose)
        print('smoothing')
        # waypoints = waypoints_not_smooth
        waypoints = smooth_path(waypoints_not_smooth, world_map)
        self.plot_map(world_map, tree, waypoints, waypoints, radius)
        
        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map):
        # extend tree by randomly selecting pose and extending tree toward that pose
        p = random_pose(world_map,end_pose[2])
        vstarIndex = find_closest_configuration(p, tree) #find closest point in 'tree' to 'p'
        vstar = tree.ned[:,vstarIndex].reshape(3,1)
        unitDirection = (p-vstar)/np.linalg.norm(p-vstar)
        vplus = vstar + self.segment_length * unitDirection #plan_path(vstar, p, self.segment_length) # find point length 'D' from 'vstar' toward 'p'
        # vplus[2] = end_pose.item(2)
        # check if path from 'vstar' to 'vplus' is valid,
        collided = collision(vstar, vplus, world_map)
        # collided = False
        if not collided:
            # calculate orientation
            course = np.arctan2(unitDirection.item(1), unitDirection.item(0))
            # update cost
            cost = self.segment_length + tree.cost[vstarIndex] 
            distanceToEnd = distance(vplus,end_pose)
            connectToGoal = 0
            # add vplus as leaf to 'tree'
            tree.add(vplus, Va, course, cost, vstarIndex, connectToGoal) 
            if distanceToEnd <= self.segment_length:
                collided = collision(vplus, end_pose, world_map)
                if not collided:
                    # create leaf for end_pose
                    treelength = np.size(tree.ned,1)
                    cost = cost + distanceToEnd
                    connectToGoal = 1
                    unitDirection = (end_pose - vplus)/np.linalg.norm(end_pose - vstar)
                    course = np.arctan2(unitDirection.item(1), unitDirection.item(0))
                    tree.add(end_pose, Va, course, cost, treelength-1, connectToGoal)                
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
        #self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        draw_tree(tree, green, self.plot_window)
        # draw things to the screen
        self.plot_app.processEvents()


def smooth_path(waypoints, world_map):
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
        wp = waypoints.ned[:,j+1]
        collided = collision(ws,wp,world_map)
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


def random_pose(world_map, pd):
    # generate a random pose
    pose = np.random.uniform(0.0,1.5*world_map.city_width,3).reshape((3,1))
    pose[2] = pd
    return pose


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(end_pose-start_pose)
    return d


def collision(start_pose, end_pose, world_map):
    safetyMargin = 70 # needs to be less than 100
    numberofPoints = int(np.linalg.norm(end_pose-start_pose) * 3)
    collision_flag = False
    # check to see of path from start_pose to end_pose colliding with map
    pathPoints = points_along_path(start_pose,end_pose,numberofPoints)
    # see if any buildings within a bounding box
    startN = start_pose.item(0)
    startE = start_pose.item(1)
    endN = end_pose.item(0)
    endE = end_pose.item(1)
    bN = world_map.building_north_east[:,0]
    bE = world_map.building_north_east[:,0]
    bNE = world_map.building_north_east.copy()
    bufferLength = safetyMargin + world_map.building_width*0.5
    if startN < endN and startE < endE: # start/end
        NE = np.array([endN + (bufferLength) , endE + (bufferLength)]) #<
        SW = np.array([startN - (bufferLength) , startE - (bufferLength)]) #>
        temp1 = bNE<NE
        temp2 = bNE>SW
        temp3 = np.concatenate((temp1, temp2),axis=1)
        temp33 = np.all(temp3,axis=1)
        nearbuildings = np.argwhere(temp33)
    elif startN >= endN and startE < endE: # start\end
        NE = np.array([startN + (bufferLength), endE + (bufferLength)]) #<
        SW = np.array([endN - (bufferLength), startE - (bufferLength)]) #>
        temp1 = bNE<NE
        temp2 = bNE>SW
        temp3 = np.concatenate((temp1, temp2),axis=1)
        temp33 = np.all(temp3,axis=1)
        nearbuildings = np.argwhere(temp33)
    elif startN >= endN and startE >= endE: # end/start
        NE = np.array([startN + (bufferLength) , startE + (bufferLength)]) #<
        SW = np.array([endN - (bufferLength) , endE - (bufferLength)]) #>
        temp1 = bNE<NE
        temp2 = bNE>SW
        temp3 = np.concatenate((temp1, temp2),axis=1)
        temp33 = np.all(temp3,axis=1)
        nearbuildings = np.argwhere(temp33)
    elif startN < endN and startE >= endE: # end\start
        NE = np.array([endN + (bufferLength) , startE + (bufferLength)]) #<
        SW = np.array([startN - (bufferLength) , endE - (bufferLength)]) #>
        temp1 = bNE<NE
        temp2 = bNE>SW
        temp3 = np.concatenate((temp1, temp2),axis=1)
        temp33 = np.all(temp3,axis=1)
        nearbuildings = np.argwhere(temp33)
    else:
        raise Exception("Missed a case, revisit conditional logic")
    heightmap = world_map.building_height.flatten()
    if nearbuildings.size > 0: # check if any buildings were found in bounding box
        for point in pathPoints: # check each point
            point =point.reshape(1,2)
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


def height_above_ground(world_map, pointInd):
    # find the altitude of point above ground level
    h_agl = world_map[pointInd.item(0)]
    return h_agl


def points_along_path(start_pose, end_pose, N):
    # returns points along path separated by Del
    pointsN = np.linspace(start_pose.item(0),end_pose.item(0),N)
    pointsE = np.linspace(start_pose.item(1),end_pose.item(1),N)
    points = np.array([pointsN,pointsE]).T
    return points


def draw_tree(tree, color, window):
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = R @ tree.ned
    for i in range(points.shape[1]):
        line_color = np.tile(color, (2, 1))
        parent = int(tree.parent.item(i))
        line_pts = np.concatenate((column(points, i).T, column(points, parent).T), axis=0)
        line = gl.GLLinePlotItem(pos=line_pts,
                                 color=line_color,
                                 width=2,
                                 antialias=True,
                                 mode='line_strip')
        window.addItem(line)


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
