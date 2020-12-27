from math import inf
from utils.Nodes import *
from utils.RobotArm import *


class AStar():
    def __init__(self, startingNode, goalNode, solver):
        self.__startingNode = startingNode
        self.__goalNode = goalNode
        self.__solver = solver

    def solve(self):
        closestList = []
        openList = []
        openList.append(self.__startingNode)

        iteration = 0

        while openList:
            state = self.__getMinimumFunctionNode(openList)

            if state.isGoal():
                return state, iteration

            iteration += 1

            for child in self.__solver.getSucessors(state):
                child.upgateGcost(state)
                child.computeHcost(self.__goalNode)
                child.updateFunctionValue()

                if not self.__checkChildInOpenList(openList, child):
                    if not self.__checkNodeInClosedList(closestList, child):
                        openList.append(child)
                        child.setParent(state)

            closestList.append(state)

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
