
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import math
import time
import vrep

RRobot = 17.7
Clearance = 12.3
extra_sum = RRobot + Clearance

vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 1700, 5)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

errorCode, bot_handle = vrep.simxGetObjectHandle(clientID, 'Turtlebot2', vrep.simx_opmode_oneshot_wait)

if clientID != -1:  # check if client connection successful
	print('Connected to remote API server')

else:
	print('Connection not successful.Exiting.')
	exit(0)

def path_rectangle(x1,x2,y1,y2,extra_sum):
    vertices = [
        (x1-extra_sum,y1-extra_sum),
        (x1-extra_sum,y2+extra_sum),
        (x2+extra_sum,y2+extra_sum),
        (x2+extra_sum,y1-extra_sum)
    ]

    P = Path(vertices)
    patch = patches.PathPatch(P,facecolor='black',lw =2)


    return P,patch

def path_circle(center,radious,extra_sum):
    #P= Circle(center,radious)
    patch = patches.Circle(center,radious+extra_sum,facecolor='black',lw=2)

    return patch

def boundary(x1,x2,y1,y2):
    vertices = [
        (x1 , y1 ),
        (x1 , y2 ),
        (x2 , y2 ),
        (x2 , y1 )
    ]

    P = Path(vertices)
    patch = patches.PathPatch(P, facecolor='black', lw=2)

    return P, patch

def check(Cordinates,Path):
    Z = Path.contains_point(Cordinates)
    return Z


def isInside(circle_x, circle_y, rad, x, y):
    if ((x - circle_x) * (x - circle_x) +
            (y - circle_y) * (y - circle_y) <= rad * rad):
        return True
    else:
        return False

def Workspace():

    fig = plt.figure()
    ax = fig.add_subplot(111)
    p1, patch1 = path_rectangle(149.95, 309.73, 750.1, 910,extra_sum)
    ax.add_patch(patch1)
    p2, patch2 = path_rectangle(438,529,315,498, extra_sum)
    ax.add_patch(patch2)
    p3, patch3 = path_rectangle(529,712,265,341, extra_sum)
    ax.add_patch(patch3)
    p4, patch4 = path_rectangle(474, 748, 35, 187, extra_sum)
    ax.add_patch(patch4)
    p5, patch5 = path_rectangle(685,1110,0,35, extra_sum)
    ax.add_patch(patch5)
    p6, patch6 = path_rectangle(927,1110,35,111, extra_sum)
    ax.add_patch(patch6)
    p7, patch7 = path_rectangle(779,896,35,93, extra_sum)
    ax.add_patch(patch7)
    p8, patch8 = path_rectangle(1052,1110,187,304, extra_sum)
    ax.add_patch(patch8)
    p9, patch9 = path_rectangle(784.5,936.5,267,384, extra_sum)
    ax.add_patch(patch9)
    p10, patch10 = path_rectangle(1019,1110,362.5,448.5, extra_sum)
    ax.add_patch(patch10)
    p11, patch11 = path_rectangle(1052,1110,448.5,565.5, extra_sum)
    ax.add_patch(patch11)
    p12, patch12 = path_rectangle(744,1110,621,697, extra_sum)
    ax.add_patch(patch12)
    p13, patch13 = path_rectangle(832,918,827,1010, extra_sum)
    ax.add_patch(patch13)
    p14, patch14 = path_rectangle(983,1026,919,1010, extra_sum)
    ax.add_patch(patch14)
    patch15 = path_circle((149.95,830.05),79.95,extra_sum)
    ax.add_patch(patch15)
    patch16 = path_circle((309.73,830.05), 79.95, extra_sum)
    ax.add_patch(patch16)
    patch17 = path_circle((390,965), 40.5, extra_sum)
    ax.add_patch(patch17)
    patch18 = path_circle((438,736), 40.5, extra_sum)
    ax.add_patch(patch18)
    patch19 = path_circle((438,274), 40.5, extra_sum)
    ax.add_patch(patch19)
    patch20 = path_circle((390,45), 40.5, extra_sum)
    ax.add_patch(patch20)
    p21,patch21 = boundary(0,1110,0,extra_sum)
    ax.add_patch(patch21)
    p22, patch22 = boundary(0, extra_sum, 0, 1011)
    ax.add_patch(patch22)
    p23, patch23 = boundary(0, 1110, 1011-extra_sum, 1011)
    ax.add_patch(patch23)
    p24, patch24 = boundary(1110-extra_sum, 1110, 0, 1011)
    ax.add_patch(patch24)
    ax.set_xlim(0, 1110)
    ax.set_ylim(0, 1010)



    P = p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,patch15,patch16,patch17,patch18,patch19,patch20,p21,p22,p23,p24

    return P

def isvalid(Cordinates,p):

    Z1 = p[0].contains_point(Cordinates)
    Z2 = p[1].contains_point(Cordinates)
    Z3 = p[2].contains_point(Cordinates)
    Z4 = p[3].contains_point(Cordinates)
    Z5 = p[4].contains_point(Cordinates)
    Z6 = p[5].contains_point(Cordinates)
    Z7 = p[6].contains_point(Cordinates)
    Z8 = p[7].contains_point(Cordinates)
    Z9 = p[8].contains_point(Cordinates)
    Z10 = p[9].contains_point(Cordinates)
    Z11 = p[10].contains_point(Cordinates)
    Z12 = p[11].contains_point(Cordinates)
    Z13 = p[12].contains_point(Cordinates)
    Z14 = p[13].contains_point(Cordinates)
    Z15 = isInside(149.95,830.05,79.95+extra_sum,Cordinates[0],Cordinates[1])
    Z16 = isInside(309.73,830.05, 79.95 + extra_sum, Cordinates[0], Cordinates[1])
    Z17 = isInside(390,965, 40.5 + extra_sum, Cordinates[0], Cordinates[1])
    Z18 = isInside(438,736,40.5 + extra_sum, Cordinates[0], Cordinates[1])
    Z19 = isInside(438,274, 40.5 + extra_sum, Cordinates[0], Cordinates[1])
    Z20 = isInside(390,45.0, 40.5 + extra_sum, Cordinates[0], Cordinates[1])
    Z21 = p[20].contains_point(Cordinates)
    Z22 = p[21].contains_point(Cordinates)
    Z23 = p[22].contains_point(Cordinates)
    Z24 = p[23].contains_point(Cordinates)

    Z = Z1 or Z2 or Z3 or Z4 or Z5 or Z6 or Z7 or Z8 or Z9 or Z10 or Z11 or Z12 or Z13 or Z14 or Z15 or Z16 or Z17 or Z18 or Z19 or Z20 or Z21 or Z22 or Z23 or Z24

    return Z


class Planning:

    def __init__(self, x, y, theta, cost, ID, Parent, Hdist):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.ID = ID
        self.Parent = Parent
        self.Hdist = Hdist
        self.RWheel = 0
        self.LWheel = 0


def unique_index(x, y):
    Index = x*150 + y * 90  # Provides a unique identity to the point set , which saves the time and computation during verification.

    return Index


def Calculate_Path(explored_set, Final_Points):
    List_X = [Final_Points.x]
    List_Y = [Final_Points.y]
    ParentID = Final_Points.Parent
    List_Node = []
    List_Node.append(Final_Points)

    while ParentID != -1:
        Node = explored_set[ParentID]
        List_Node.append(Node)
        ParentID = Node.Parent


    return List_Node


def Generate_neighbors(Current_N, Final_Points, extra_sum,p):
    radious_wheel = 3.8
    L = 23
    X_initial = Current_N.x
    Y_initial = Current_N.y
    T_initial = Current_N.theta
    X_Final = Final_Points.x
    Y_Final = Final_Points.y
    cost_initial = Current_N.cost

    # delta_time = 0.025
    # RPM1 = 20
    # RPM2 = 15

    delta_time = 0.013
    RPM1 = 40
    RPM2 = 50

    Moving_Action = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]

    Neighbor_Values = []

    for i in range(0, 8):

        right_velocity = ((2 * math.pi) / 60) * (Moving_Action[i][1])
        left_velocity = ((2 * math.pi)/ 60) * (Moving_Action[i][0])


        for k in range(100):
            T_Final = T_initial + (right_velocity - left_velocity) * delta_time * radious_wheel / L
            Move_X = X_initial + (left_velocity + right_velocity) * math.cos(T_Final) * delta_time * (radious_wheel / 2)
            Move_Y = Y_initial + (left_velocity + right_velocity) * math.sin(T_Final) * delta_time * (radious_wheel / 2)
            Coordinates = (int(Move_X), int(Move_Y))

            if isvalid(Coordinates,p) == True:
                break
            T_initial = T_Final
            X_initial = Move_X
            Y_initial = Move_Y

        if k ==99:
            Move_Y = int(Move_Y)
            Move_X = int(Move_X)
            Coordinates = (Move_X,Move_Y)
            if isvalid(Coordinates,p) ==True:
                 continue

            Move_Cost = cost_initial + distance_heuristic(Move_X, Move_Y, X_initial, Y_initial)
            Final_Cost = Move_Cost + distance_heuristic(Move_X, Move_Y, X_Final, Y_Final)

            if (extra_sum < Move_X < 1110) and (extra_sum < Move_Y < 1010):
                Condition = isvalid(Coordinates,p)
                if Condition == False:
                    UID = unique_index(Move_X, Move_Y)
                    Neighbor_Node = Planning(Move_X, Move_Y, T_Final, Move_Cost, UID, Current_N.ID, Final_Cost)
                    Neighbor_Node.RWheel = right_velocity
                    Neighbor_Node.LWheel = left_velocity
                    Neighbor_Values.append(Neighbor_Node)

    return Neighbor_Values


def distance_heuristic(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


def Astar_algo(StartPoints, GoalPoints, p):
    start_time = time.time()
    plt.plot(StartPoints[0], StartPoints[1], 'rx')
    plt.plot(GoalPoints[0], GoalPoints[1], 'rx')


    Check1 = isvalid(StartPoints,p)
    Check2 = isvalid(GoalPoints,p)

    if Check1 == True or Check2 == True or StartPoints[0] > 1110 or StartPoints[1] > 1010 or GoalPoints[0] > 1110 or GoalPoints[1] > 1010:
        print('The Points are either in the Obstacle Space or out of the Workspace.Exiting.')
        exit(0)

    explored_set = dict()
    future_set = dict()
    show_animation = list()

    Hdist_source = distance_heuristic(StartPoints[0], StartPoints[1], GoalPoints[0], GoalPoints[1])
    Source_Points = Planning(StartPoints[0], StartPoints[1], 0, 0, 0, -1, Hdist_source)
    Final_Points = Planning(GoalPoints[0], GoalPoints[1], 0, 0, 0, 0, 0)

    Source_Points.ID = unique_index(Source_Points.x, Source_Points.y)
    future_set[Source_Points.ID] = Source_Points

    while len(future_set) != 0:

        ID_Current = min(future_set, key=lambda i: future_set[i].Hdist)
        Current_N = future_set[ID_Current]

        show_animation.append((Current_N.x, Current_N.y))
        x, y = zip(*show_animation)

        del future_set[ID_Current]
        explored_set[ID_Current] = Current_N


        if ((round(Current_N.x,2) - Final_Points.x)**2 + (round(Current_N.y,2) - Final_Points.y)**2 - 500)<= 0:

            Final_Points.Parent = Current_N.Parent
            Final_Points.cost = Current_N.cost

            break

        if len(show_animation) % 100 == 0:
            plt.plot(x, y, 'g.')
            plt.pause(0.1)
            show_animation.clear()

        Neighbors = Generate_neighbors(Current_N, Final_Points, extra_sum,p)


        for Neighbor in Neighbors:


            if Neighbor.ID in explored_set:
                continue

            if Neighbor.ID in future_set:
                if future_set[Neighbor.ID].cost > Neighbor.cost:
                    future_set[Neighbor.ID].cost = Neighbor.cost
                    future_set[Neighbor.ID].Hdist = Neighbor.Hdist
                    future_set[Neighbor.ID].Parent = ID_Current

            else:
                future_set[Neighbor.ID] = Neighbor

    List_Node = Calculate_Path(explored_set, Final_Points)
    List_X = [node.x for node in List_Node]
    List_Y = [node.y for node in List_Node]
    plt.plot((List_X),(List_Y), c='b')
    final = time.time()
    print('Found goal in --->', final - start_time)
    plt.show()

    Data = [[Node.LWheel,Node.RWheel] for Node in List_Node]
    Data.reverse()
    print(Data)


    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_left_joint', vrep.simx_opmode_oneshot_wait)
    errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_right_joint',
                                                             vrep.simx_opmode_oneshot_wait)

    for g in (range(len(Data))):
        Vel_Left = Data[g][0]
        Vel_Right = Data[g][1]

        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, Vel_Left, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, Vel_Right, vrep.simx_opmode_streaming)

        time.sleep(5)


    Vel_Left = 0
    Vel_Right = 0
    errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, Vel_Left, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, Vel_Right, vrep.simx_opmode_streaming)


    return "Execution Successful"


p = Workspace()

print('Enter Start Node.')
X_start = int(input("Enter x coordinate in cms"))
Y_start = int(input("Enter y coordinate in cms"))
print('Enter Goal Node.')
X_Goal = int(input("Enter x coordinate in cms"))
Y_Goal = int(input("Enter x coordinate in cms"))

vrep.simxSetObjectPosition(clientID,bot_handle,-1,[((X_start/100)-5.1870), ((Y_start/100)-4.8440),(0.06)],vrep.simx_opmode_oneshot_wait)
Result = Astar_algo((X_start,Y_start),(X_Goal,Y_Goal),p)

print(Result)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
vrep.simxFinish(clientID)

