
#********************************************************************#
#####  UNIVERSITY OF MARYLAND                                #########
#####  ENPM661 - Planning for Autonomous Robots              #########
#####  PROJECT 5 - GROUP 28                                  #########
#####  Member 1: PRASANNA MARUDHU BALASUBRAMANIAN            #########
#####  Member 2: CHAYAN KUMAR PATODI                         #########
#####  SPRING 2019                                           #########
#********************************************************************#

#Importing the Modules
import pygame
import time
import numpy as np
import math
from scipy.stats import linregress
from scipy.spatial.distance import euclidean

#This function measures the path length
def path_length(List_X, List_Y):
    length = 0
    for i in range(len(List_X) - 1):
        v = [List_X[i], List_Y[i]]
        w = [List_X[i + 1], List_Y[i + 1]]
        length += euclidean(v, w)
    return length


#Defining class with all the function that is been used in this project
class Algorithm_BFS:
    def __init__(self, space, rhombus, rect, cir,rect2, rect3, rect4,rect5, rect6,circ2,circ3, rect7):
        self.boundarypoints = space
        self.rhombus = rhombus
        self.rect5 = rect5
        self.rect6 = rect6
        self.rect = rect
        self.rect2 = rect2
        self.rect3 = rect3
        self.rect4 = rect4
        self.rect7 = rect7

        self.cir = cir
        self.circ2 = circ2
        self.circ3 = circ3
        self.obs_array = []
        self.disp = pygame.display.set_mode(self.boundarypoints)
        self.env_space()

    x = 0
    y = 0

    def find_path(self, goal):
        gp = goal

        List_X = [goal[0]]
        List_Y = [goal[1]]
        while (gp != self.sp):

            List_X.append(gp[0])
            List_Y.append(gp[1])

            pygame.draw.line(self.disp, (255, 0, 0), gp, gp,2 )

            pygame.display.update()
            gp = self.node_info[gp]

        #print("gp_listx", List_X, "gp_listy", List_Y)
        Total_dist = path_length(List_X, List_Y)
        print('This BFS path length is ' + str(Total_dist) + ' meters')
        print (" ")
        Scaled = pygame.transform.scale(self.disp, (640,480))
        pygame.image.save(Scaled, "bfs.jpeg")

        rand = input('Path Found')
        exit()


    def crosscheck(self, point, line_segment):

        val = self.worldpoints(point, self.boundarypoints, line_segment[0]) != self.worldpoints(point,
                                                                                                self.boundarypoints,
                                                                                                line_segment[
                                                                                                    1]) and self.worldpoints(
            point, line_segment[0], line_segment[1]) != self.worldpoints(self.boundarypoints, line_segment[0],
                                                                         line_segment[1])
        if val == True:
            return 1
        if val == False:
            return 0

    def check_rect5(self, point):
        #Plan is to check the crosscheck
        variable = 0
        for x in range(len(rect5)):
            variable = variable + self.crosscheck(point, [rect5[x-1], rect5[x]])
            #print variable
        if (variable % 2) == 0:
            return False
        else:
            return True

    def check_rect6(self, point):
        # Plan is to check the crosscheck
        variable = 0
        for x in range(len(rect6)):
            variable = variable + self.crosscheck(point, [rect6[x - 1], rect6[x]])
            # print variable
        if (variable % 2) == 0:
            return False
        else:
            return True


    def newnode(self, check):
        PointList = []
        #print check
        if (check[1] - 1 >= 0):
            PointList.append((check[0], check[1] - 1))
        if (check[1] + 1 < 250):
            PointList.append((check[0], check[1] + 1))
        if (check[0] - 1 >= 0):
            PointList.append((check[0] - 1, check[1]))
        if (check[0] + 1) < 250:
            PointList.append((check[0]  + 1, check[1]))
        if (check[0] + 1 < 250) and (check[1] - 1 >= 0):
            PointList.append((check[0] + 1, check[1] - 1))
        if (check[0] + 1 < 250) and (check[1] + 1 < 250):
            PointList.append((check[0] + 1, check[1] + 1))
        if (check[0] - 1 >= 0) and (check[1] - 1 >= 0):
            PointList.append((check[0] - 1, check[1] - 1))
        if (check[0] - 1 >= 0) and (check[1] + 1 < 250):
            PointList.append((check[0] - 1, check[1] + 1))

        return PointList

    def env_space(self):
        #This is for the gui
        blank_world = np.zeros(self.boundarypoints, dtype='int')
        for x in range(self.boundarypoints[0]):
            for y in range(self.boundarypoints[1]):
                if self.space_obs((x,y)):
                    blank_world[x][y] = 1
                    pygame.draw.line(self.disp, (0, 0, 255), (x,y), (x,y))

        pygame.display.update()


    def ObstacleCircle(self, point):
        if math.sqrt((point[0] - ((self.cir[0] + self.cir[2])/2))**2 + (point[1] - ((self.cir[1] + self.cir[3])/2))**2) <= 32 :
            return True
        else:
            return False

    def ObstacleCirclec2(self, point):
        if math.sqrt((point[0] - ((self.circ2[0] + self.circ2[2])/2))**2 + (point[1] - ((self.circ2[1] + self.circ2[3])/2))**2) <= (28.5) :
            return True
        else:
            return False

    def ObstacleCirclec3(self, point):
        if math.sqrt((point[0] - ((self.circ3[0] + self.circ3[2])/2))**2 + (point[1] - ((self.circ3[1] + self.circ3[3])/2))**2) <= 20 :
            return True
        else:
            return False

    def check_rect7(self, point):
        if (point[0] >= self.rect7[0][0]) & (point[0] <= self.rect7[1][0]) & (point[1] >= self.rect7[0][1]) & (point[1] <= self.rect7[1][1]) :
            return True
        else:
            return False


    def imp_bfs(self, point, epoint):
        start_time = time.time()
        self.sp = point
        closed_point = []
        queue = []
        self.node_info = {}
        current_point = point
        while(current_point != epoint):
            newnode = self.newnode(current_point)
            node_valid = [c for c in newnode if hash(c) not in closed_point]
            node_valid = [c for c in node_valid if c not in queue]

            node_allowed = []

            for obj in node_valid:
                temp = self.space_obs(obj)
                #print temp
                if temp == False:
                    node_allowed.append(obj)
            #print node_allowed
            for obj in node_allowed:
                pygame.draw.line(self.disp, (255, 255, 0), obj, obj)
                pygame.display.update()
                self.node_info[obj] = current_point

            #print node_allowed

            queue.extend(node_allowed)
            closed_point.append(hash(current_point))
            #print queue
            #print closed_point
            #print(closed_point)
            try:
                current_point = queue.pop(0)

                #print current_point
                #print len(closed_point)
                #rand = raw_input()
            except:
                break
        total_time = time.time() - start_time
        print ('This BFS took ' + str(total_time) + ' seconds')

        self.find_path(epoint)

    def check_rect(self, point):
        if (point[0] >= self.rect[0][0]) & (point[0] <= self.rect[1][0]) & (point[1] >= self.rect[0][1]) & (point[1] <= self.rect[1][1]) :
            return True
        else:
            return False

    def check_rect2(self, point):
        if (point[0] >= self.rect2[0][0]) & (point[0] <= self.rect2[1][0]) & (point[1] >= self.rect2[0][1]) & (point[1] <= self.rect2[1][1]) :
            return True
        else:
            return False

    def check_rect3(self, point):
        if (point[0] >= self.rect3[0][0]) & (point[0] <= self.rect3[1][0]) & (point[1] >= self.rect3[0][1]) & (point[1] <= self.rect3[1][1]) :
            return True
        else:
            return False

    def check_rect4(self, point):
        if (point[0] >= self.rect4[0][0]) & (point[0] <= self.rect4[1][0]) & (point[1] >= self.rect4[0][1]) & (point[1] <= self.rect4[1][1]) :
            return True
        else:
            return False


    def rhombus(self, point):
        #Plan is to check the crosscheckion
        variable = 0
        for x in range(len(rhombus)):
            variable = variable + self.crosscheck(point, [rhombus[x-1], rhombus[x]])
            #print variable
        if (variable % 2) == 0:
            return False
        else:
            return True


    def space_obs(self, point):
        #checking the point if it's in the obstacle
        if self.check_rect(point) or self.ObstacleCircle(point) or self.ObstacleCirclec2(point) or self.ObstacleCirclec3(point) or self.check_rect2(point) or self.check_rect3(point) or self.check_rect4(point) or self.check_rect5(point) or self.check_rect6(point)  or self.check_rect7(point):
            return True
        else:
            return False



    def worldpoints(self,A,B,C):
        #checking the range
        return ((C[1]-A[1]) * (B[0]-A[0])) > ((B[1] - A[1]) * (C[0] - A[0]))



if __name__ == '__main__':

    print(" ")
    bound = 149
    start_point = 180, 245 - 100  # -100+24
    goal_point = 130, -100 + 45  # 250-100

    # goal_point = (250,149-149)
    start_point = (start_point[0], bound - start_point[1])
    goal_point = (goal_point[0], bound - goal_point[1])
    rhombus = [(120, 249 - 55), (158, 249 - 51), (165, 249 - 89), (188, 249 - 51), (168, 249 - 14), (145, 249 - 14)]
    circ = (10, 250 - 210 - 32, 10 + 64, 250 - 210 + 32)
    circ2 = (135, (250 - 40 - 28.5), 135 + 57, (250 - 40 + 28.5))
    circ3 = (10, (250 - 120 - 20), 10 + 40, (250 - 120 + 20))
    square = [(10, 250 - 12 - 53), (10 + 42, 250 - 10)]
    rect2 = [(69, 250 - 12 - 50), (69 + 50, 250 - 12)]
    rect3 = [(69, 250 - 80 - 80), (69 + 125, 250 - 80)]
    rect4 = [(120, 250 - 210 - 30), (120 + 55, 250 - 210)]
    rect5 = [(200, 250 - 210), (200, 250 - 240), (245, 250 - 240), (245, 250 - 120), (245 - 25, 250 - 120),
             (245 - 25, 250 - 210), (200, 250 - 210)]
    rect6 = [(245 - 40, 250 - 55), (245 - (20), 250 - 95), (245, 250 - 55), (245 - (20), 250 - 15)]
    rect7 = [(25, 250 - 100 - 40), (25 + 30, 250 - 100)]

    # Defining space
    space = (250, 250)
    Obstaclespace = Algorithm_BFS(space, rhombus, square, circ, rect2, rect3, rect4, rect5, rect6, circ2, circ3, rect7)

    # Analysing if the given points is in the obstacle space
    if Obstaclespace.space_obs(start_point):
        print("The Start point you entered is in the obstacles pace")
        print("Please give a different start point")
        exit()
    if Obstaclespace.space_obs(goal_point):
        print("The Goal point you entered is in the obstacles pace")
        print("Please give a different goal point")
        exit()
    Obstaclespace.imp_bfs(start_point, goal_point)
    print(goal_point)
    Obstaclespace.find_path(goal_point)



