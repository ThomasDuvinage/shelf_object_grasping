from utils.PlaceFinder import *
from utils.RobotArm import *
import numpy as np
import sys


np.set_printoptions(threshold=sys.maxsize)


class Solver(PlaceFinder):
    def __init__(self, shelf_size_x, shelf_size_y, precision):
        """ This class permits to find the right strategy to reach the goal object without touching any other object.

        Args:
            shelf_size_x (int): x size of the shelf in mm
            shelf_size_y (int): y size of the shelf in mm
            precision (int): number of mm per pixel
        """
        self.__graph = []
        self.goal = None

        self.objectRadiusProximity = 70  # defined depending on the arm's size

        self.precision = precision

        super().__init__(shelf_size_x, shelf_size_y, precision)

    def createGraph(self, node_array):
        """ Cette methode permet de generer le graph en fonction des objets dans l'espace

        Un noeud ne peut avoir qu'un fils par contre il peut avoir plusieurs parents.
        """
        for node in node_array:
            if not node.isGoal():
                child, _ = node.getClosestNode(node_array)

                child.setParent(node)
                node.setChild(child)
            else:
                self.goal = node

        self.__graph = node_array

        return self.goal

    def checkDirectConnectivity(self, robotArm=None):
        """INFO : Cette fonction va changer car on recuperera la position du bras grace a ros

        Args:
            robotArm (Object): robot arm object
        """

        if robotArm:
            print("robot arm check")
            for node in self.__graph:
                if not self.__isCollide(robotArm, node):
                    node.setRobotArmAccessibility()

        else:
            print("direct check")
            for node in self.__graph:
                if not node.isGoal():
                    if not self.__isCollide(self.goal, node):
                        node.setDirectTrajecory()

        return self.__graph

    def defineObjectToMove(self):
        """Cette methode definit la liste des objets a bouger pour atteindre l'object goal.
            En verifiant auparavant si l'objet goal ne peut pas etre atteint directement.
        """
        pass

    def __getDistanceToClosestObjectsFromPoint(self, point):
        closest_node = None
        min_dist = 100000

        for obj in self.__graph:
            if(min_dist > (obj.getDistanceTo(point) + obj.size/2)):
                closest_node = obj
                min_dist = obj.getDistanceTo(point) + obj.size/2

        return closest_node, min_dist

    def __isCollide(self, starting_node, ending_node):
        """ This method is based on the Bresenham algorithm

            This algorithm is well known for drawing lines between two points in a grid.

            In our case, we use this algorithm to link two poses with a line and for each cell of this line, we check if the arm can pass without touching any other object.
        Args:
            starting_node (Node or RobotArm):
            ending_node (Node):
        """

        sx, sy, dx, dy = 0, 0, 0, 0

        x, y = starting_node.x, starting_node.y

        if(x < ending_node.x):
            sx = 1
        else:
            sx = -1

        if(y < ending_node.y):
            sy = 1
        else:
            sy = -1

        dx = abs(x - ending_node.x)
        dy = abs(y - ending_node.y)
        e = dx - dy

        while(x != ending_node.x or y != ending_node.y):
            e2 = e * 2

            if e2 > - dy:
                e -= dy
                x += sx

            if e2 < dx:
                e += dx
                y += sy

            if(x != ending_node.x or y != ending_node.y):
                node, distanceToClosestNode = self.__getDistanceToClosestObjectsFromPoint([
                    x, y, 0])

                if node is not ending_node and node is not starting_node:
                    if distanceToClosestNode < self.objectRadiusProximity:
                        return True

        return False
