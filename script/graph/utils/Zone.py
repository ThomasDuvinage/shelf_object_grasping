from utils.Point3D import *
import numpy as np

from math import sqrt, pow, acos


class Zone(Point3D):

    def __init__(self, name, coordinate, size):
        super().__init__(coordinate)

        self.name = name
        self.size = size

        self._child = []
        self._parent = None

        self.gCost = 0
        self.hCost = 0
        self.functionValue = 0

    def resetChild(self):
        self._child = []

    def setChild(self, nodeChild):
        self._child.append(nodeChild)

    def getChild(self):
        return self._child

    def isGoal(self):
        return False

    def resetParent(self):
        self._parent = None

    def setParent(self, parentChild):
        self._parent = parentChild

    def getParent(self):
        return self._parent

    def computeHcost(self, goal):
        self.hCost = self.getDistanceToNode(goal)
        return self.hCost

    def computeVirtualGcost(self, parent):
        print("parent = ", parent.gCost)
        v = parent.gCost + self.getDistanceToNode(parent)
        print(v)
        return parent.gCost + self.getDistanceToNode(parent)

    def upgateGcost(self, parent):
        # here we can take in acount the mecanical constraints of the robot arm
        # to do so, we can add to the distance the angle between the two nodes

        self.gCost = parent.gCost + \
            self.getDistanceToNode(parent) * 0.8  # distance

        vectorPC = [self.x - parent.x, self.y - parent.y]
        vectorPy = [0, 1 - parent.y]

        unitVecPC = vectorPC / np.linalg.norm(vectorPC)
        unitVecPy = vectorPy / np.linalg.norm(vectorPy)

        dot_product = np.dot(unitVecPC, unitVecPy)

        angle = np.arccos(dot_product) * 1.5

        self.gCost += angle

        return self.gCost

    def isAtTheSamePositionAs(self, node):
        if self.x == node.x:
            if self.y == node.y:
                return True
        return False

    def updateFunctionValue(self):
        self.functionValue = self.hCost + self.gCost

    def getFunctionValue(self):
        self.functionValue = self.hCost + self.gCost
        return self.functionValue

    def getDistanceTo(self, datapoint):
        return sqrt(pow((datapoint[0]-self.x), 2) + pow((datapoint[1]-self.y), 2) + pow((datapoint[2]-self.z), 2))

    def getDistanceToNode(self, node):
        return sqrt(pow((node.x-self.x), 2) + pow((node.y-self.y), 2) + pow((node.z-self.z), 2))

    def __str__(self):
        """Equivalent of toString(). it permits to display all the info concerning the node
        """
        rep = "Zone :\n"
        rep += "    name: " + self.name + "\n"
        rep += "    size: " + str(self.size) + "\n"

        if(self._child):
            for child in self._child:
                rep += "    child: " + child.name + "\n"
        else:
            rep += "    child: " + str(self._child) + "\n"

        if(self._parent):
            rep += "    parent: " + self._parent.name + "\n"
        else:
            rep += "    parent: " + str(self._parent) + "\n"

        return rep
