from math import sqrt, pow


class Node():
    def __init__(self, name='', coordinate=[], isGoal=False, size=0, child=None, parent=None):
        self.x, self.y, self.z = coordinate[0]['x'], coordinate[0]['y'], coordinate[0]['z']
        self.name = name
        self.size = size

        self.__child = child

        if not parent:
            self.__parent = []
        else:
            self.__parent = parent

        self.__isGoal = isGoal
        self.__directTractory = False
        self.__robotArmAccessible = False

    def isRobotArmAccessible(self):
        if self.__robotArmAccessible:
            return True

        return False

    def setRobotArmAccessibility(self):
        self.__robotArmAccessible = True

    def setChild(self, nodeChild):
        self.__child = nodeChild

    def getChild(self):
        return self.__child

    def setParent(self, parentChild):
        self.__parent.append(parentChild)

    def getParent(self):
        return self.__parent

    def setGoal(self):
        self.__isGoal = True

    def isGoal(self):
        return self.__isGoal

    def setDirectTrajecory(self):
        self.__directTractory = True

    def isDirectTrajectory(self):
        return self.__directTractory

    def getDistanceTo(self, datapoint):
        return sqrt(pow((datapoint[0]-self.x), 2) + pow((datapoint[1]-self.y), 2) + pow((datapoint[2]-self.z), 2))

    def getDistanceToNode(self, node):
        return sqrt(pow((node.x-self.x), 2) + pow((node.y-self.y), 2) + pow((node.z-self.z), 2))

    def getClosestNode(self, array):
        closest_node = None
        min_dist = 100000

        for node in array:
            if node is not self and not node.isGoal():
                if(min_dist > self.getDistanceToNode(node)):
                    closest_node = node
                    min_dist = self.getDistanceToNode(node)

        return closest_node

    def isAccessible(self, radius):
        for parent in self.__parent:
            if self.getDistanceToNode(parent) < radius:
                return False

        if self.__child and self.getDistanceToNode(self.__child) < radius:
            return False

        return True

    def __str__(self):
        rep = "Object :\n"
        rep += "    name: " + self.name + "\n"
        rep += "    size: " + str(self.size) + "\n"
        rep += "    isdirectTraj: " + str(self.__directTractory) + "\n"
        rep += "    isRobotAccessible: " + \
            str(self.__robotArmAccessible) + "\n"

        rep += "    isGoal: " + str(self.__isGoal) + "\n\n"
        if(self.__child):
            rep += "    child: " + self.__child.name + "\n"
        else:
            rep += "    child: " + str(self.__child) + "\n"

        if(self.__parent):
            for parent in self.__parent:
                rep += "    parent: " + parent.name + "\n"
        else:
            rep += "    parent: " + str(self.__parent) + "\n"

        return rep
