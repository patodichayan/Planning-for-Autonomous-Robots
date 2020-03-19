
#********************************************************************#
#####  UNIVERSITY OF MARYLAND                                #########
#####  ENPM661 - Planning for Autonomous Robots              #########
#####  PROJECT 5 - GROUP 28                                  #########
#####  Member 1: PRASANNA MARUDHU BALASUBRAMANIAN            #########
#####  Member 2: CHAYAN KUMAR PATODI                         #########
#####  SPRING 2019                                           #########
#********************************************************************#

#Importing the Modules
from scipy.spatial.distance import euclidean
from math import *
import math, sys, pygame, random

from pygame import *
import time

class Points_Node(object):
    def __init__(self, pts, old_parent):
        super(Points_Node, self).__init__()
        self.pts = pts
        self.old_parent = old_parent

def obstacle_define(section):  # initialized the obstacle
    global Imped_rectangle
    Imped_rectangle = []

    if (section == 1):
        Imped_rectangle.append(pygame.Rect((10, 190), (48, 50)))
        Imped_rectangle.append(pygame.Rect((80, 188), (50, 53)))
        Imped_rectangle.append(pygame.Rect((80, 250 - 160), (105, 70)))
        Imped_rectangle.append(pygame.Rect((110, 10), (60, 30)))
        Imped_rectangle.append(pygame.Rect((200, 10), (45, 30)))
        Imped_rectangle.append(pygame.Rect((225, 10), (20, 115)))
        Imped_rectangle.append(pygame.Rect((25, 250 - 145), (30, 40)))

        # circle obstacles
        circObs_c = pygame.draw.circle(screen, green, (165, 215), 25)
        circObs.append(circObs_c)
        circObs_c2 = pygame.draw.circle(screen, green, (40, 40), 32)
        circObs.append(circObs_c2)
        circObs_c3 = pygame.draw.circle(screen, green, (25, 125), 20)
        circObs.append(circObs_c3)

        # rhombus obstacle
        rhomb = pygame.draw.polygon(screen, green, [(215, 200), (230, 170), (245, 200), (230, 230)])
        rhombObs.append(rhomb)

    for rect in Imped_rectangle:
        pygame.draw.rect(screen, green, rect)

def main():

    #Initializing the time and ValInit
    start = time.time()
    ValInit = False

    # Initializing the Node
    Points_Nodes = []
    prev()

    #Initializing the Firstpoint
    Firstpoint = Points_Node(None, None)
    # Initializing the FinalInit
    FinalInit = False
    # Initializing the Pt_goal
    Pt_goal = Points_Node(None, None)
    # Initializing the Present_state
    Present_state = 'init'

    global def_val

    while True:
        if Present_state == 'init':
            print("Click on the window")
            pygame.display.set_caption('Select Starting Point and then Goal Point')
            fpsClock.tick(10)
        elif Present_state == 'goalFound':
            currPoints_Node = goalPoints_Node.old_parent
            pygame.display.set_caption('Goal Reached')

            List_X = [currPoints_Node.old_parent.pts[0]]
            List_Y = [currPoints_Node.old_parent.pts[1]]

            EndTime = time.time()
            print (" ")
            print ('This RRT_Algorithm took ' + str(EndTime-start) + ' seconds')


            while currPoints_Node.old_parent != None:
                #print ("currPoints_Node.old_parent.x",currPoints_Node.old_parent.x)
                #print("currPoints_Node.0x", currPoints_Node[0])
                #print("currpoint",currPoints_Node.pts[0])
                List_X.append([currPoints_Node.old_parent.pts[0]])
                List_Y.append([currPoints_Node.old_parent.pts[1]])


                pygame.draw.line(screen,red,currPoints_Node.pts,currPoints_Node.old_parent.pts)
                currPoints_Node = currPoints_Node.old_parent
            Flag = True

            Total_dist = path_length(List_X, List_Y)
            print('This RRT path length is ' + str(Total_dist) + ' meters')
            #Scaled = pygame.transform.scale(screen, (640, 480))
            #pygame.image.save(screen, "bfs.jpeg")

        elif Present_state == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif Present_state == 'buildTree':
            def_val = def_val+1
            pygame.display.set_caption('Algorithm RRT')
            if def_val < count_Points_Nodes:
                Consec_point = False
                while Consec_point == False:
                    rand = tree_points()
                    parentPoints_Node = Points_Nodes[0]
                    for p in Points_Nodes:
                        if Length(p.pts,rand) <= Length(parentPoints_Node.pts,rand):
                            newPoint = Func_step_to(p.pts,rand)
                            if obstacle_coll(newPoint) == False:
                                parentPoints_Node = p
                                Consec_point = True

                newPoints_Node = Func_step_to(parentPoints_Node.pts,rand)
                Points_Nodes.append(Points_Node(newPoints_Node, parentPoints_Node))
                pygame.draw.line(screen,cyan,parentPoints_Node.pts,newPoints_Node)

                if Col_circle(newPoints_Node, Pt_goal.pts, bound_radius):
                    Present_state = 'goalFound'

                    goalPoints_Node = Points_Nodes[len(Points_Nodes)-1]


            else:
                print("Enter more node points")
                return;



        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:

                if Present_state == 'init':
                    if ValInit == False:
                        Points_Nodes = []
                        if obstacle_coll(e.pos) == False:


                            Firstpoint = Points_Node((180,250-245), None)
                            Points_Nodes.append(Firstpoint) # Start in the center
                            ValInit = True
                            pygame.draw.circle(screen, red, Firstpoint.pts, bound_radius)
                    elif FinalInit == False:
                        #print('goal pts set: '+str(e.pos))
                        if obstacle_coll(e.pos) == False:
                            Pt_goal = Points_Node((130,250-45),None)
                            FinalInit = True
                            pygame.draw.circle(screen, green, Pt_goal.pts, bound_radius)
                            Present_state = 'buildTree'
                else:
                    Present_state = 'init'
                    ValInit = False
                    FinalInit = False
                    prev()

        pygame.display.update()
        fpsClock.tick(10000)

#Distance between the points
def Length(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))





def obstacle_coll(p):    #check if pts obstacle_coll with the obstacle
    for rect in Imped_rectangle:
        if rect.collidepoint(p) == True:
            return True

    for rect in circObs:
        if rect.collidepoint(p)==True:
            return True

    for rect in rhombObs:
        if rect.collidepoint(p) == True:
            return True

    return False





def path_length(List_X, List_Y):
    length = 0
    for i in range(len(List_X) - 1):
        v = [List_X[i], List_Y[i]]
        w = [List_X[i + 1], List_Y[i + 1]]
        length += euclidean(v, w)
    return length

#Defining color
white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0,180,105

def prev():
    global def_val
    screen.fill(white)
    obstacle_define(Check_lev)
    def_val = 0


#Delta Level
op_delt = 10.0



Check_lev = 1
#Radius of goal
bound_radius = 2

def Func_step_to(p1,p2):
    if Length(p1,p2) < op_delt:
        return p2
    else:
        angle_theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + op_delt*cos(angle_theta), p1[1] + op_delt*sin(angle_theta)

#Length of vertex formed
vertex_len = 1

#Maximum number of Points_Nodes that can be explored
count_Points_Nodes = 10000

#initializing the pygame environment
pygame.init()

def tree_points():
    while True:
        pointy = random.random()*Dimension_X, random.random()*Dimension_Y
        nil_col = obstacle_coll(pointy)
        if nil_col == False:
            return pointy

#Variable initialization
def_val = 0
Imped_rectangle = []
circObs =[]
rhombObs=[]

#Defining the dimension space
Dimension_X = 250
Dimension_Y = 250

#Defining the window size
windowSize = [Dimension_X, Dimension_Y]
fpsClock = pygame.time.Clock()

#Initializing the screen space
screen = pygame.display.set_mode(windowSize)

def Col_circle(p1, p2, radius):
    distance = Length(p1,p2)
    if (distance <= radius):
        return True
    return False


#Main Function
if __name__ == '__main__':
    main()
    







