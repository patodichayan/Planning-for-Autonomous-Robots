#----------------------------------------------------------------------------------------------#
# Name     : Chayan Kumar Patodi.
# UID      : 116327428
# Subject  : ENPM 661 - Planning for Autonomous Robots
# Project2 : Implementation of Dijkstra and A-star Planning Algorithms.
#----------------------------------------------------------------------------------------------#

import numpy as np
import matplotlib.pyplot as plt
import math

RCircle = 15  #Radious of the Circle.
CCenter = (190,130)
           
A = 15   #Semi-Major Axis of the Ellipse.
B = 6   #Semi-Major Axis of the Ellipse.
ECenter = (140,120)

# The half plane region is defined by combination of line equations. 
# Each line equation is defined by the standard  "ax+by+c=0"

# Array "A" represents the constants "a" in the linear equation with respect to an obstacle.
# Array "B" represents the constants "b" in the linear equation with respect to an obstacle.
# Array "C" represents the constants "c" in the linear equation with respect to an obstacle.

Points = np.array([[125,56],[150,15],[173,15],[163,52],[170,90],[193,52]])

#We break down the concave polygon into 2 convex polygon , to use the Half plane method.
# Part 1

m1 = (Points[1][1] - Points[0][1])/(Points[1][0] - Points[0][0]) # Slope
c1 = Points[0][1] - m1*Points[0][0]                            #Y- Intercept.

m2 = (Points[2][1] - Points[1][1])/(Points[2][0] - Points[1][0])
c2 = Points[1][1] - m2*Points[1][0]

m3 = (Points[3][1] - Points[2][1])/(Points[3][0] - Points[2][0])
c3 = Points[2][1] - m3*Points[2][0]

m4 = (Points[0][1] - Points[3][1])/(Points[0][0] - Points[3][0])
c4 = Points[3][1] - m4*Points[3][0]

A2 = [m1, m2, -m3, -m4]
B2 = [-1 ,-1, 1, 1]
C2 = [c1 ,c2 ,-c3 ,-c4]

#Part 2

m5 = (Points[5][1] - Points[4][1])/(Points[5][0] - Points[4][0])
c5 = Points[4][1] - m5*Points[4][0]

m6 = (Points[4][1] - Points[3][1])/(Points[4][0] - Points[3][0])
c6 = Points[3][1] - m6*Points[3][0]

m7 = (Points[3][1] - Points[2][1])/(Points[3][0] - Points[2][0])
c7 = Points[2][1] - m7*Points[2][0]

m8 = (Points[2][1] - Points[5][1])/(Points[2][0] - Points[5][0])
c8 = Points[5][1] - m8*Points[5][0]

A3 = [-m5 ,-m6, m7, m8]
B3 = [1, 1 ,-1 ,-1]
C3 = [-c5, -c6, c7, c8]



# Similarly for the Rectangle.

A1 = [-1, 1, 0, 0] 
B1 = [0 ,0, -1, 1]
C1 = [50, -100, 67.5, -112.5]

class Planning:

     def __init__(self,x,y,cost,ID,Parent,Hdist):
     
         self.x = x
         self.y = y
         self.cost = cost
         self.ID = ID
         self.Parent = Parent
         self.Hdist = Hdist
         
         
def obstacle_space(Range,RRobot,Clearance,Resolution):

    x_initial = round(Range[0]/Resolution)
    y_initial = round(Range[1]/Resolution)
    x_final = round(Range[2]/Resolution)
    y_final = round(Range[3]/Resolution)
    
    O_Points = set()
    
    RRobot = (RRobot/Resolution)
    Clearance = (Clearance/Resolution)
    Sum = RRobot + Clearance
    
    c9  = c1/Resolution - Sum*(np.sqrt(1 + m1**2))
    c10 = c2/Resolution - Sum*(np.sqrt(1 + m2**2)) 
    c11 = c3/Resolution - Sum*(np.sqrt(1 + m3**2)) 
    c12 = c4/Resolution + Sum*(np.sqrt(1 + m4**2))
    c13 = c5/Resolution + Sum*(np.sqrt(1 + m5**2)) 
    c14 = c6/Resolution + Sum*(np.sqrt(1 + m6**2))
    c15 = c7/Resolution - Sum*(np.sqrt(1 + m7**2)) 
    c16 = c8/Resolution - Sum*(np.sqrt(1 + m8**2))
            
    C4 = [c9, c10, -c11, -c12]
    C5 = [-c13, -c14, c15, c16]
    
    #Forming the Workspace:
    
    Workspace = plt.Rectangle((x_initial,y_initial),x_final,y_final,color='w')
    plt.gca().add_patch(Workspace)
    plt.grid(True)
    
    
    for x in np.arange(x_initial,x_final):
        for y in np.arange(y_initial,y_final):
    
            Current_Points = (x,y)
            
    
    #Forming the Circle:
           
            if ((x - (CCenter[0]/Resolution))**2) + ((y - (CCenter[1]/Resolution))**2) - (((RCircle/Resolution) + Sum)**2) <= 0:
                O_Points.add(Current_Points)
                
                
    #Forming the Ellipse:
            elif ((x-(ECenter[0]/Resolution))**2)/(((A/Resolution) + Sum)**2) + ((y-(ECenter[1]/Resolution))**2)/(((B/Resolution) + Sum)**2) - 1 <= 0:
                
                O_Points.add(Current_Points)
                
    #Forming the Rectangle:
    
            elif  (A1[0]*x + B1[0]*y + (C1[0]/Resolution) - Sum) <= 0 and (A1[1]*x + B1[1]*y + (C1[1]/Resolution) - Sum) <= 0 and (A1[2]*x + B1[2]*y + (C1[2]/Resolution) - Sum) <= 0 and (A1[3]*x + B1[3]*y + (C1[3]/Resolution) - Sum) <= 0 :
            
                O_Points.add(Current_Points)
    
    #Forming the Polygon:
                        
                     
            elif (A2[0]*x + B2[0]*y + C4[0])  <= 0 and (A2[1]*x + B2[1]*y + C4[1]) <= 0 and (A2[2]*x + B2[2]*y + (C2[2]/Resolution)) <= 0 and (A2[3]*x + B2[3]*y + C4[3]) <=0 :
    
                O_Points.add(Current_Points)
                
            elif (A3[0]*x + B3[0]*y + C5[0])  <= 0 and (A3[1]*x + B3[1]*y + C5[1]) <= 0 and (A3[2]*x + B3[2]*y + (C3[2]/Resolution)) <= 0 and (A3[3]*x + B3[3]*y + C5[3]) <= 0:
    
                O_Points.add(Current_Points)
            
            elif x < Sum or y < Sum or x > (250/Resolution) - Sum or y >(150/Resolution) - Sum:
                O_Points.add(Current_Points) 
     
    for i in O_Points:
        plt.plot(i[0],i[1],'b.')
        
    return O_Points

def unique_index(x,y):
    
    Index = x + y*90  #Provides a unique identity to the point set , which saves the time and computation during verification.
    
    return Index

def Calculate_Path(explored_set,Final_Points):
    
    List_X = [Final_Points.x]
    List_Y = [Final_Points.y]
    ParentID = Final_Points.Parent
    
    while ParentID != -1:
        Node = explored_set[ParentID]
        List_X.append(Node.x)
        List_Y.append(Node.y)
        ParentID = Node.Parent
        
    return List_X,List_Y
    
         
def Generate_neighbors(Current_N,O_Points,Resolution):

    X = Current_N.x
    Y = Current_N.y
    cost = Current_N.cost
    Moving_Action = [(1, 0, 1), (0, 1, 1),(-1, 0, 1), (0, -1, 1), (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),(1,-1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    
    Neighbor_Values = []
    
    for i in range(0,8):
    
         Move_X = X + Moving_Action[i][0]
         Move_Y = Y + Moving_Action[i][1]
         Move_Cost = cost + Moving_Action[i][2]
         Coordinates = (Move_X,Move_Y)
         
         if Move_X in range(0,round(251/Resolution)) and Move_Y in range(0,round(151/Resolution)):
         
             if Coordinates not in O_Points:
                 UID = unique_index(Move_X,Move_Y)
                 Neighbor_Node = Planning(Move_X,Move_Y,Move_Cost,UID,Current_N.ID,0)
                 Neighbor_Values.append(Neighbor_Node)
                 
    return Neighbor_Values            

    
def dijksta_algo(StartPoints,GoalPoints,RRobot,Clearance,Resolution):

    plt.plot(StartPoints[0],StartPoints[1],'rx')
    plt.plot(GoalPoints[0],GoalPoints[1],'rx')
    O_Points = obstacle_space((0,0,251,151),RRobot,Clearance,Resolution)
    
    if StartPoints in O_Points or StartPoints[0]>250/Resolution or StartPoints[1]>150/Resolution or GoalPoints in O_Points or GoalPoints[0]>250/Resolution or GoalPoints[1]>150/Resolution:
    
        print('The Points are either in the Obstacle Space or out of the Workspace.Exiting.')
        exit(0)
        
        
    explored_set = dict()
    future_set = dict()
    show_animation = list()
    
        
    Source_Points = Planning(StartPoints[0],StartPoints[1],0,0,-1,0)
    Final_Points = Planning(GoalPoints[0],GoalPoints[1],0,0,0,0)
    
    Source_Points.ID = unique_index(Source_Points.x,Source_Points.y)
    future_set[Source_Points.ID] = Source_Points
    
    
    while len(future_set) != 0:
    
        ID_Current = min(future_set,key=lambda i: future_set[i].cost)
        Current_N = future_set[ID_Current]
        
        show_animation.append((Current_N.x,Current_N.y))
        x,y = zip(*show_animation)
        
        del future_set[ID_Current]
        explored_set[ID_Current] = Current_N
        
        if Current_N.x == Final_Points.x and Current_N.y == Final_Points.y:
            Final_Points.Parent = Current_N.Parent
            Final_Points.cost = Current_N.cost
            plt.plot(x,y,'yo')
            break
            
        if len(show_animation) % (100/Resolution) == 0:
            plt.plot(x,y,'yo')
            plt.pause(0.0001)
            show_animation.clear()
            
       
        
        Neighbors = Generate_neighbors(Current_N,O_Points,Resolution)
        
        for Neighbor in Neighbors:
            if Neighbor.ID in explored_set:
                continue
                
                
            if Neighbor.ID in future_set:    
                if future_set[Neighbor.ID].cost > Neighbor.cost:
                    future_set[Neighbor.ID].cost = Neighbor.cost
                    future_set[Neighbor.ID].Parent = ID_Current
                    
                    
            else:
                future_set[Neighbor.ID] = Neighbor
                
    
    List_X,List_Y = Calculate_Path(explored_set,Final_Points)
    for i in range(len(List_X)):
        plt.plot(List_X[i],List_Y[i],'k.') 
        
    plt.show()                      
        
def distance_heuristic(x1,y1,x2,y2):

    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return distance
    
    
def Generate_neighbors_A(Current_N,Goal_N,O_Points,Resolution):

    X = Current_N.x
    Y = Current_N.y
    Goal_X = Goal_N.x
    Goal_Y = Goal_N.y
    cost = Current_N.cost
    Moving_Action = [(1, 0, 1), (0, 1, 1),(-1, 0, 1), (0, -1, 1), (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),(1,-1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    
    Neighbor_Values = []
    
    for i in range(0,8):
    
         Move_X = X + Moving_Action[i][0]
         Move_Y = Y + Moving_Action[i][1]
         Move_Cost = cost + Moving_Action[i][2]
         Hdist = Move_Cost + distance_heuristic(Move_X,Move_Y,Goal_X,Goal_Y)
         Coordinates = (Move_X,Move_Y)
         if Move_X in range(0,round(251/Resolution)) and Move_Y in range(0,round(151/Resolution)):
         
             if Coordinates not in O_Points:
                 UID = unique_index(Move_X,Move_Y)
                 Neighbor_Node = Planning(Move_X,Move_Y,Move_Cost,UID,Current_N.ID,Hdist)
                 Neighbor_Values.append(Neighbor_Node)
                 
    return Neighbor_Values 
         
         
def Astar_algo(StartPoints,GoalPoints,RRobot,Clearance,Resolution):

    
    plt.plot(StartPoints[0],StartPoints[1],'rx')
    plt.plot(GoalPoints[0],GoalPoints[1],'rx')
    O_Points = obstacle_space((0,0,251,151),RRobot,Clearance,Resolution)
    
    if StartPoints in O_Points or StartPoints[0]>250/Resolution or StartPoints[1]>150/Resolution or GoalPoints in O_Points or GoalPoints[0]>250/Resolution or GoalPoints[1]>150/Resolution:
    
        print('The Points are either in the Obstacle Space or out of the Workspace.Exiting.')
        exit(0)
        
        
    explored_set = dict()
    future_set = dict()
    show_animation = list()
    
    Hdist_source = distance_heuristic(StartPoints[0],StartPoints[1],GoalPoints[0],GoalPoints[1])
    Source_Points = Planning(StartPoints[0],StartPoints[1],0,0,-1,Hdist_source)
    Final_Points = Planning(GoalPoints[0],GoalPoints[1],0,0,0,0)
    
    Source_Points.ID = unique_index(Source_Points.x,Source_Points.y)
    future_set[Source_Points.ID] = Source_Points
    
    while len(future_set) != 0:
    
        ID_Current = min(future_set , key=lambda i: future_set[i].Hdist)
        Current_N = future_set[ID_Current]
        
        show_animation.append((Current_N.x,Current_N.y))
        x,y = zip(*show_animation)
        
        if Current_N.x == Final_Points.x and Current_N.y == Final_Points.y:
        
            Final_Points.Parent = Current_N.Parent
            Final_Points.cost = Current_N.cost
            plt.plot(x,y,'y.')
            break
        
        if len(show_animation) % (100/Resolution) == 0:
            plt.plot(x,y,'g.')
            plt.pause(0.0001)
            show_animation.clear()
            
        del future_set[ID_Current]
        explored_set[ID_Current] = Current_N
        
        Neighbors =  Generate_neighbors_A(Current_N,Final_Points,O_Points,Resolution)
        
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
                
    List_X,List_Y =  Calculate_Path(explored_set,Final_Points)
     
    for i in range(len(List_X)):
         plt.plot(List_X[i],List_Y[i],'k.')       
         
         
    plt.show()               
                
                                  
        
def main():

    print("Select your choice of robot.")
    print("Press 1 for Point Robot.")
    print("Press 2 for Rigid Body Robot.")
    I = int(input("Make your selection :"))
    if I ==1:
       RRobot = 0
       Clearance = 0
       
    elif I ==2:
       print("Enter Radious of your Robot.")
       RRobot = int(input())
       print("Enter Clearance for your Robot.")
       Clearance = int(input())
       
    else:
       print("Invalid Choice. Exiting.")
       exit(0)   
    
    print("Enter the start point x-coordinate")
    X_start = int(input())
    print("Enter the start point y-coordinate")
    Y_start = int(input())
    print("Enter the goal point x-coordinate")
    X_goal = int(input())
    print("Enter the goal point y-coordinate")
    Y_goal = int(input())
    print("Enter the Resolution")
    Resolution = int(input())
    
    print("Select your choice of Path Planning Algorithm.")
    print("Press 1 for Dijkstra Algorithm")
    print("Press 2 for A-Star Algorithm")
    S = int(input("Make you selection:"))
    print("Usually starts within a minute.")
    if S == 1:
    
        Validity = dijksta_algo((X_start,Y_start),(X_goal,Y_goal),RRobot,Clearance,Resolution)
        if Validity == "The Points are either in the Obstacle Space or out of the Workspace.Exiting.":
            print(Validity)
            exit(0)
            
    elif S == 2:
    
        Validity = Astar_algo((X_start,Y_start),(X_goal,Y_goal),RRobot,Clearance,Resolution)
        if Validity == "The Points are either in the Obstacle Space or out of the Workspace.Exiting.":
            print(Validity)
            exit(0)
            
    else:
        print("Option Not Valid. Exiting Code.")
        exit(0)        
             
if __name__ == '__main__':

    main()



