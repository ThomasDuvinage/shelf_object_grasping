from utils.Point3D import *

from math import sqrt, pow


class Zone(Point3D):

    def __init__(self, name, coordinate, size):
        super().__init__(coordinate)

        self.name = name
        self.size = size

        self._child = []
        self._parent = None

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
