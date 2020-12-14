import numpy as np
import sys
import copy as cp
from math import inf

from solver.PlaceFinder import *
from utils.RobotArm import *
from utils.Nodes import *
from utils.FreeZone import *
from solver.AStar import *


class Solver(PlaceFinder):
    def __init__(self, shelf_size_x, shelf_size_y, precision, graph, goal):
        """ This class permits to find the right strategy to reach the goal object without touching any other object.

        Args:
            shelf_size_x (int): x size of the shelf in mm
            shelf_size_y (int): y size of the shelf in mm
            precision (int): number of mm per pixel
        """
        self.__graph = graph
        self.goal = goal

        self.__shelf_size_x = shelf_size_x
        self.shelf_size_y = shelf_size_y

        self.__objectRadiusProximity = 150  # defined depending on the arm's size

        self.precision = precision

        super().__init__(self.__graph, shelf_size_x,
                         shelf_size_y, precision)

    def getSucessors(self, currentNode):
        """ Cette methode permet de generer le graph en fonction des objets dans l'espace

        Un noeud ne peut avoir qu'un fils par contre il peut avoir plusieurs parents.
        """

        if isinstance(currentNode, Node):
            freeSpaces = self.findPlace(currentNode)

            if freeSpaces:
                for zone in freeSpaces:
                    if not self.__isCollide(currentNode, zone):
                        newSpace = cp.deepcopy(zone)

                        currentNode.setChild(newSpace)

        if isinstance(currentNode, RobotArm):
            for node in self.__graph:
                if node.name[:-1] == "RobotArm-Path-Point":
                    new_node = cp.deepcopy(node)
                    new_node.resetChild()

                    currentNode.setChild(new_node)

        else:
            for node in self.__graph:
                if node.name is not currentNode.name:
                    if not self.__isCollide(currentNode, node):
                        new_node = cp.deepcopy(node)
                        new_node.resetChild()

                        currentNode.setChild(new_node)
        return currentNode.getChild()

    def defineObjectToMove(self, robotArm, algo_name, occurence_test=True):
        """Cette methode definit la liste des objets a bouger pour atteindre l'object goal.
            En verifiant auparavant si l'objet goal ne peut pas etre atteint directement.
        """

        objectsToMove = []

        solution = None
        nb_iterations = 0

        if algo_name == "BFS":
            solution, nb_iterations = self.breath_first_search(
                robotArm, occurence_test=occurence_test)
        elif algo_name == "DFS":
            solution, nb_iterations = self.depth_first_search(
                robotArm, occurence_test=occurence_test)
        elif algo_name == "A*":
            AStarSolver = AStar(robotArm, self.goal, self)

            solution, nb_iterations = AStarSolver.solve()

        if solution:
            print("Solver : Solution found")

        objectsToMove.append(solution)
        parent = solution.getParent()

        while parent:
            objectsToMove.append(parent)

            parent = parent.getParent()

        objectsToMove.reverse()

        return objectsToMove, nb_iterations

    def breath_first_search(self, robotArm, occurence_test=True):
        frontier = list()
        explored = []
        frontier.append(robotArm)

        i = 0

        while frontier:
            state = frontier.pop(0)

            if state.isGoal():
                return state, i

            children = self.getSucessors(state)

            for child in children:
                if not occurence_test or (child.name not in explored):
                    child.setParent(state)
                    frontier.append(child)
                    if occurence_test:
                        explored.append(child.name)

            i += 1

        return None

    def depth_first_search(self, robotArm, occurence_test=True):
        frontier = list()
        explored = []
        frontier.append(robotArm)

        i = 0

        while frontier:
            state = frontier.pop()

            explored.append(state)

            if state.isGoal():
                return state, i

            children = self.getSucessors(state)

            for child in children:
                if not occurence_test or (child.name not in explored):
                    child.setParent(state)
                    frontier.insert(0, child)
                    if occurence_test:
                        explored.append(child.name)

                i += 1

        return None

    def __isCollide(self, starting_node, ending_node):
        """ This method is based on the Bresenham algorithm

            This algorithm is well known for drawing lines between two points in a grid.

            In our case, we use this algorithm to link two poses with a line and for each cell of this line, we check if the arm can pass without touching any other object.
        Args:
            starting_node (Node or RobotArm):
            ending_node (Node):
        """
        x, y = int(starting_node.x), int(starting_node.y)

        if(x < ending_node.x):
            sx = 1
        else:
            sx = -1

        if(y < ending_node.y):
            sy = 1
        else:
            sy = -1

        dx = abs(x - int(ending_node.x))
        dy = abs(y - int(ending_node.y))
        e = dx - dy

        while(x != int(ending_node.x) or y != int(ending_node.y)):
            e2 = e * 2

            if e2 > - dy:
                e -= dy
                x += sx

            if e2 < dx:
                e += dx
                y += sy

            if(x != int(ending_node.x) or y != int(ending_node.y)):
                node, distanceToClosestNode = self.__getDistanceToClosestObjectsFromPoint([
                    x, y, 0])

                if node:
                    if (node.name is not ending_node.name and node.name is not starting_node.name):
                        if distanceToClosestNode < self.__objectRadiusProximity:
                            return True

        return False

    def __getDistanceToClosestObjectsFromPoint(self, point):
        """This method permits to find the closest object to a given one.

        Args:
            point (Node)

        Returns:
            [Node, float]: closest_node, min_dist
        """
        closest_node = None
        min_dist = inf

        for obj in self.__graph:
            distance = obj.getDistanceTo(point) + obj.size/2
            if(min_dist > distance):
                closest_node = obj
                min_dist = distance

        return closest_node, min_dist

    def resetNodesChild(self):
        for node in self.__graph:
            node.resetChild()
            node.resetParent()

    def newPoseObjectToMove(self, solutions):
        freeSpaceAccessible = []
        newPosAvailable = []
        i = 2
        while not solution[i].isGoal():
            #newPosAvailable.append("Object1 pose possible"+str(i-1))
            "trouver tous les espaces libre"
            freeSpace = self.findPlace(solution[i])

            "ne regarder que les espaces accessible"
            for point in freeSpace:
                self.__isCollide(solution[1], point)
                freeSpaceAccessible.append(point)

            "retirer l'objet qui nous interesse du graph"
            for compt, objectToretire in enumerate(self.__graph):
                if objectToretire.name == solution[i].name:
                    tamponObj = objectToretire
                    self.__graph.pop(compt)

            "Tester si les valeurs conviennent pour deplacer l'objet"
            for pointA in freeSpaceAccessible:
                self.__graph.append(pointA)
                if self.__isCollide(solution[1], solution[i+1]):
                    if self.addValue(pointA, newPosAvailable):
                        newPosAvailable.append(pointA)
                self.__graph.pop()

            self.__graph.append(tamponObj)

            i += 1
        for al in newPosAvailable:
            solution.append(al)
        return newPosAvailable

    def addValue(self, pointA, posAvailable):
        add = True
        for point in posAvailable:
            if point == pointA:
                add = False
        return add
