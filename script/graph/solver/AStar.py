from math import inf
from utils.Nodes import *
from utils.RobotArm import *


class AStar():
    def __init__(self, startingNode, goalNode, solver):
        self.__startingNode = startingNode
        self.__goalNode = goalNode
        self.__solver = solver

    def solve(self):
        """The goal of this function is to find the best way to reach the goal object using A* algorithm.
            Which means respect robot arm mechanical constraints and passing the minimum number of objects

        Returns:
            [Node, int]: [Goal Node, iterations]
        """
        closestList = []
        openList = []
        openList.append(self.__startingNode) # add starting node to openList 

        iteration = 0

        
        while openList: #while open list is not empty 
            state = self.__getMinimumFunctionNode(openList) #get the node in openList with the minimum f function value 

            if state.isGoal(): #if current state is goal 
                return state, iteration #return state and iterations

            for child in self.__solver.getSucessors(state): #for every successors of state
                child.upgateGcost(state) #chil compute G cost with state as parent
                child.computeHcost(self.__goalNode) #compute heuristic 
                child.updateFunctionValue() #update the f function 

                #if child is in closedList or OpenList or G function value of child is higher to the virtual computed child g cost 
                if not self.__checkChildInOpenList(openList, child):
                    if not self.__checkNodeInClosedList(closestList, child):
                        openList.append(child) #then add child to openList 
                        child.setParent(state) #set state as child's parent

            closestList.append(state)#add current state to closedList
            iteration += 1 #increment interation

        return None, iteration

    def __getMinimumFunctionNode(self, nodeList):
        minimum = inf
        best_node = None
        index = 0

        for i, node in enumerate(nodeList):
            if node.getFunctionValue() < minimum:
                best_node = node
                index = i
                minimum = node.getFunctionValue()

        return nodeList.pop(index)

    def __checkNodeInClosedList(self, closedList, child):
        for node in closedList:
            if child.name == node.name:
                if child.getFunctionValue() > node.getFunctionValue():
                    return True

        return False

    def __checkChildInOpenList(self, openList, child):
        for node in openList:
            if child.name == node.name:
                if child.getFunctionValue() > node.getFunctionValue():
                    return True

        return False
