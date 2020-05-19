import rospy  # this is the module required for all simulation communication
import numpy as np
import time
from wheel_control.msg import wheelSpeed  # this is a required module for the drive communication
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

rospy.init_node("controller")

class WheelController:

    def __init__(self):
        self.wheel_pub = rospy.Publisher("/gazebo_wheelControl/wheelSpeedTopic", wheelSpeed, queue_size=1)

    def drive_wheels(self, left, right):
        # type: (float, float) -> None
        # left and right are numbers between -1 and 1
        msg = wheelSpeed()
        msg.left = left
        msg.right = right
        msg.wheelMode = 0
        self.wheel_pub.publish(msg)
        #print(msg)


class LaserListener:

    def __init__(self):
        self.laserSub = rospy.Subscriber("/leddar/leddarData", LaserScan, self.laser_callback, queue_size=1)
        self.laserRanges = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]

    def laser_callback(self, msg):
        # type: (LaserScan) -> None
        self.laserRanges = msg.ranges


class LocationHeading:

    def __init__(self):
        self.fixSub = rospy.Subscriber("/fix/metres", Point, self.fix_callback, queue_size=1)
        self.headingSub = rospy.Subscriber("/heading",Float32, self.heading_callback, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0

    def fix_callback(self, msg):
        # type: (Point) -> None
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def heading_callback(self, msg):
        # type: (Float32) -> None
        self.heading = msg.data


def search(grid, init, goal, cost, heurstic, size=41):

    closed = np.zeros(shape=(size, size))
    closed[init[0], init[1]] = 1

    action = np.zeros(shape=(size, size))
    x = init[0]
    y = init[1]
    g = 0
    f = g + heuristic[init[0], init[0]] #second one maybe init[1]
    cell = [[f, g, x, y]]

    found = False
    resign = False

    while not found and not resign:
        if len(cell) == 0:
            return "FAIL"
        else:
            cell.sort()
            cell.reverse()
            next = cell.pop()
            x = next[2]
            y = next[3]
            g = next[1]

            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < size and y2 >= 0 and y2 < size:
                        if closed[x2, y2] == 0 and grid[x2, y2] == 0:
                            g2 = g + cost
                            f2 = g2 + heuristic[x2, y2]
                            cell.append([f2, g2, x2, y2])
                            closed[x2, y2] = 1
                            action[x2, y2] = i

    invpath = []
    x = int(goal[0])
    y = int(goal[1])
    invpath.append([x, y])
    while x != init[0] or y != init[1]:
        x2 = x - delta[int(action[x][y])][0]
        y2 = y - delta[int(action[x][y])][1]
        x = x2
        y = y2
        invpath.append([x, y])

    path = []
    for i in range(len(invpath)):
        path.append(invpath[len(invpath) - 1 - i])
    #print("ACTION MAP")
    for i in range(len(action)):
        pass
        #print(action[i])

    return path


locHead  = LocationHeading()
laser = LaserListener()
wheel = WheelController()

np.set_printoptions(threshold=np.nan, linewidth=200) # numpy print properties to see the full grid.

SIZE =25 # make this odd so that the rover is initiated in the centre of a grid. 
map = np.zeros(shape=(SIZE, SIZE)) # makes a sizexsize grid.
count = 0 # first few seconds can bug out for the reciever so this is just a buffer.
angle_between_lasers = 0.052359877 # rad, found in the rover's base code.
rover_y = 0.0
rover_x = 0.0
iteration = 0
follownum = 0
goal = [20, 20] 
cost = 1
heuristic = np.zeros(shape=(SIZE, SIZE))    
delta = [[-1, -1], [-1, 0],[-1, 1],[1, 0],[1, -1],[1, 1],[0, 1],[0, -1]]
initiate = 0

for i in range(SIZE):
    for j in range(SIZE):
        heuristic[i, j] = round(np.sqrt((i - goal[0])**2 + (j - goal[1])**2))


while not rospy.is_shutdown():
    iteration += 1

    if count == 0:
        time.sleep(2)
        count += 1

    left_most_laser = locHead.heading - (angle_between_lasers/2) - (7*angle_between_lasers)
    
    if locHead.x != rover_x: 
        map[int(round(rover_x)), int(round(rover_y))] = 0
        rover_x = (locHead.x + (SIZE-1)/2)
        rover_y = (locHead.y + (SIZE-1)/2)
        map[int(round(rover_x)), int(round(rover_y))] = 2
    elif locHead.y != rover_y:
        map[int(round(rover_x)), int(round(rover_y))] = 0
        rover_y = (locHead.y + (SIZE-1)/2)
        rover_x = (locHead.x + (SIZE-1)/2)
        map[int(round(rover_x)), int(round(rover_y))] = 2
    
    distances = laser.laserRanges
    
    for distance in distances: #iterate through the ranges list
        angle = left_most_laser + ((distances.index(distance))*angle_between_lasers)
        obstacle_x = rover_x + distance*(np.cos(angle))  
        obstacle_y = rover_y + distance*(np.sin(angle))
        if obstacle_x < SIZE and obstacle_y < SIZE and obstacle_x > 0 and obstacle_y > 0:
            map[int(obstacle_x), int(obstacle_y)] = 1

    #print(map) # prints the lidar map with obstacles populated

    #search iterated over the lidar map
    a = search(map, [int(rover_x), int(rover_y)], goal, cost, heuristic, SIZE)

    #print(a) prints the list of coordinates the rover needs to follow.
    
    # the following code below will place the coordinates generated by the search onto a copy of the lidar map
    # to visualize the path
    '''
    temp_moves = np.copy(map)
    for x,y in a:
        temp_moves[x,y] = '3'
    
    temp_moves[goal[0],goal[1]] = '4'
    print(temp_moves)
    '''

    if round(rover_x) == round(goal[0]) and round(rover_y) == round(goal[1]):
        print("Destination Achieved!")

    

    # if nothing in step2 is present, it means that the search has determined the goal has been reached.
    try:
        step = a[0]
        step2 = a[1]
        xo = float(step2[0]) - float(step[0])  
        yo = float(step2[1]) - float(step[1])
    except:
        print("Goal Reached.")
        break

    facing = locHead.heading

    # angle calculation for which way the rover needs to drive

    if xo == 0 and yo > 0:
        angle = 1.56
    elif xo == 0 and yo < 0:
        angle = 4.71
    elif yo == 0 and xo > 0:
        angle = 0.01
    elif yo == 0 and xo < 0:
        angle = 3.14
    elif xo > 0 and yo > 0:
        angle = 0.76
    elif xo > 0 and yo < 0:
        angle = 5.52
    elif xo < 0 and yo < 0:
        angle = 3.90
    else:
        angle = 2.38


    # basic movement - the weakest part of the program and where most time efficiency is lost.

    if round(facing, 1) == round(angle, 1):
        wheel.drive_wheels(1.0,1.0)
        #print("drive straight")
    elif (round(facing, 2) >= 5.52) and (angle <= 0.70):
        wheel.drive_wheels(-0.8, 0.8)
    elif (round(facing, 2) <= 0.80) and (angle >= 5.50):
        wheel.drive_wheels(0.8, -0.8)
    elif (round(facing, 2) > 0.6 and round(facing, 2) < 0.85) and (angle > 2.7 and angle < 3.2):
        wheel.drive_wheels(-0.8, 0.8)
    elif round(facing, 1) > round(angle, 1):
        wheel.drive_wheels(1.0, -1.0)
        #print("turn right")
    elif round(facing, 1) < round(angle, 1):
        wheel.drive_wheels(-1.0, 1.0)
        #print("turn left")
    else:
        print('should never be here')
    

    if iteration >= 17500:
        print("Simulation finished")
        break
    time.sleep(0.02) # higher sleep = more precise points but increases room for gaps. basically means that the program is iterated (0.02^-1)times/s
    
#stops the rover once the goal is reached.
for i in range(4):
    wheel.drive_wheels(0.0, 0.0)
    time.sleep(1) 








