#----------------------------------------------------------------------------------------------#
#Name     : Chayan Kumar Patodi.
#UID      : 116327428
#Subject  : ENPM 661 - Planning for Autonomous Robots
#Project1 : Find all the possible states of the 8-Puzzle starting from the given initial state.
#----------------------------------------------------------------------------------------------#

print('Authored by Chayan Kumar Patodi')


import numpy as np
import copy
import time
import collections


Goal_State = np.array([[1,2,3],[4,5,6],[7,8,0]])
Nodes = []
Info = []
NodeString = set()
ID = 1


class Puzzle:

	def __init__(self,values = None, nodeID = None, parentID = None):
		self.values = values
		self.id = nodeID
		self.parentID = parentID

	def Turn2String(self,State,count):					

		if count == 1:
		
			s = ''
			for i in State:
				for j in i:
					s += str(j)
			return s

		if count == 2:

			s = ''
			for i in State:
				for j in i:
					s += str(j) + " "
			return s
	def bfs(self):

		print('Enter the Input Matrix (Values from 0-8)')
		Input_Value = np.array([[int(j) for j in input().split()] for i in range(3)])
		global ID
		global Nodes
		Initial_Time = time.time()
		Node_Stack = collections.deque()
		Initial_Node = Puzzle(Input_Value,ID,0)
		Node_Stack.append(Initial_Node)
		Tag = 0

		while Node_Stack:

			Time_Instant = time.time()
			Current_Node = Node_Stack.popleft()
			Nodes.append(Current_Node.values)
			NodeString.add(self.Turn2String(Current_Node.values,1))
			Info.append([Current_Node.id,Current_Node.parentID])
			Tag = self.Check_GoalState(Current_Node)
			if Tag ==1:
				print("Goal State was reached.")
				print("Computational Time = "  +str(Time_Instant - Initial_Time))
				break
			
			Node_Update = self.Moving_Action(Current_Node,NodeString)
			Node_Stack.extend(Node_Update)

		if Tag ==0:
			print("No Possible Solution Found.")
			print("Computational Time = " +str (Time_Instant - Initial_Time))
			self.Output(" ",4)

	def Moving_Action(self,Parent,Total_Nodes):
		Children_Node = []
		global Info
		global ID
		x0,y0 = 0,0

		#To find the location of the "0th Tile/Blank Tile"
		for i in range(0, 3):
			for j in range(0,3):
				if Parent.values[i][j] == 0:
					x0 = i
					y0 = j


		#Move UP
		if x0 is not 0:

			Child_Node = Puzzle(copy.deepcopy(Parent.values),None,Parent.id)
			store = Child_Node.values[x0 - 1][y0]
			Child_Node.values[x0 - 1][y0] = Child_Node.values[x0][y0]
			Child_Node.values[x0][y0] = store	

			
			if self.Turn2String(Child_Node.values,1) not in Total_Nodes:
				ID += 1
				Child_Node.id = ID
				Children_Node.append(Child_Node)		
			

		#Move DOWN
		if x0 is not 2:

			Child_Node = Puzzle(copy.deepcopy(Parent.values),None,Parent.id)
			store = Child_Node.values[x0 + 1][y0]
			Child_Node.values[x0 + 1][y0] = Child_Node.values[x0][y0]
			Child_Node.values[x0][y0] = store	

			
			if self.Turn2String(Child_Node.values,1) not in Total_Nodes:
				ID += 1
				Child_Node.id = ID
				Children_Node.append(Child_Node)		
		

	        #Move LEFT
		if y0 is not 0:

			Child_Node = Puzzle(copy.deepcopy(Parent.values),None,Parent.id)
			store = Child_Node.values[x0][y0 - 1]
			Child_Node.values[x0][y0 - 1] = Child_Node.values[x0][y0]
			Child_Node.values[x0][y0] = store	

			
			if self.Turn2String(Child_Node.values,1) not in Total_Nodes:
				ID += 1
				Child_Node.id = ID
				Children_Node.append(Child_Node)		
		
		#Move RIGHT
		if y0 is not 2:

			Child_Node = Puzzle(copy.deepcopy(Parent.values),None,Parent.id)
			store = Child_Node.values[x0][y0 + 1]
			Child_Node.values[x0][y0 + 1] = Child_Node.values[x0][y0]
			Child_Node.values[x0][y0] = store	

			
			if self.Turn2String(Child_Node.values,1) not in Total_Nodes:
				ID += 1
				Child_Node.id = ID
				Children_Node.append(Child_Node)		
			

		return Children_Node


	def Check_GoalState(self,Current_Node):
		Path = []
		temp = 0
		if self.Turn2String(Current_Node.values,1) == self.Turn2String(Goal_State,1):

			print("Total number of nodes the program ran through-> "+str(len(Nodes)))
			#print(Nodes)
			print("Length of Info matrix -> "+str(len(Info)))
			#print(Info)
			C = Current_Node.id
			P  = Current_Node.parentID
			Path.append(Nodes[C - 1])
			Path.append(Nodes[P  - 1])


			while P  is not 1:

				C = [x for x in Info if P  in x][0][0]
				P  = [x for x in Info if P  in x][0][1]
				Path.append(Nodes[C - 1])
				Path.append(Nodes[P  - 1])

			
			Path.reverse()
			self.Output(Nodes, 1)
			self.Output(Path, 2)
			self.Output(Info, 3)
			#print("Node Path is ")			
			#print(Path)
			Tag =  1
			return Tag

		else:
			Tag = 0
			return Tag	

	def Output(self,Total_Nodes,count):
		
		if count == 1:

			Final_Output = open('Nodes.txt', 'w')
			for i in range(0, len(Total_Nodes)):
				Node_Pattern = Total_Nodes[i].transpose()
				string_value = self.Turn2String(Node_Pattern,2)
				Final_Output.write("%s \n" % string_value)
			Final_Output.close()

		if count == 2:

			Final_Output = open('NodePath.txt', 'w')
			for i in range(0, len(Total_Nodes)):
				Node_Pattern = Total_Nodes[i].transpose()
				string_value = self.Turn2String(Node_Pattern,2)
				Final_Output.write("%s \n" % string_value)
			Final_Output.close()

		if count == 3:

			Final_Output = open('NodeInfo.txt', 'w')
			for i in range(0, len(Total_Nodes)):
				for j in range(0, 2):
					string_value = Total_Nodes[i][j]
					Final_Output.write("%s " % string_value)
				Final_Output.write("\n")
			Final_Output.close()

		if count ==4:
			Final_Output = open('Nodes.txt','w')
			Final_Output.write(" ")
			Final_Output.close()
			Final_Output = open('NodePath.txt','w')
			Final_Output.write(" ")
			Final_Output.close()
			Final_Output = open('NodeInfo.txt','w')
			Final_Output.write(" ")
			Final_Output.close()


				
			

		
def main():
		RUN = Puzzle()
		RUN.bfs()
		print('The outputs can be found in the text file generated in the same folder.')

main()

		
		
