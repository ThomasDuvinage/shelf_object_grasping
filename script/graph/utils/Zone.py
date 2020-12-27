from utils.Point3D import *
from math import inf


class Zone(Point3D):

    def __init__(self, name, coordinate, size):
        super().__init__(coordinate)

        self.name = name
        self.size = size

        self._child = []
        self._parent = None

        self.gAngleCost = 0
        self.gDistCost = 0
        self.gObjectCost = 0
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
        d = parent.gDistCost + self.getDistanceToNode(parent)

        a = self.computeAngle(parent)
        a += parent.gAngleCost

        o = parent.gObjectCost + 1000

        return d + a + o

    def getGCost(self):
        return self.gDistCost + self.gAngleCost

    def upgateGcost(self, parent):
        # here we can take in acount the mecanical constraints of the robot arm
        # to do so, we can add to the distance the angle between the two nodes

        self.gDistCost = parent.gDistCost + \
            self.getDistanceToNode(parent)  # distance

        angle = self.computeAngle(parent)

        self.gAngleCost = parent.gAngleCost + angle

        self.gObjectCost = parent.gObjectCost + 1000

        return self.gDistCost + self.gAngleCost + self.gObjectCost

    def isAtTheSamePositionAs(self, node):
        if self.x == node.x:
            if self.y == node.y:
                return True
        return False

    def updateFunctionValue(self):
        self.functionValue = self.hCost + self.gDistCost + self.gAngleCost

    def getFunctionValue(self):
        self.functionValue = self.hCost + self.gDistCost + self.gAngleCost
        return self.functionValue

    def getDistanceTo(self, datapoint):
        return sqrt(pow((datapoint[0]-self.x), 2) + pow((datapoint[1]-self.y), 2) + pow((datapoint[2]-self.z), 2))

    def getDistanceToNode(self, node):
        return sqrt(pow((node.x-self.x), 2) + pow((node.y-self.y), 2) + pow((node.z-self.z), 2))

    def getClosestZoneFromList(self, liste):
        max_dist = inf
        b_node = None

        for element in liste:
            if max_dist > self.getDistanceToNode(element):
                max_dist = self.getDistanceToNode(element)
                b_node = element

        return b_node, max_dist

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
