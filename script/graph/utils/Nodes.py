from math import sqrt, pow
import copy as cp


class Node():

    def __init__(self, name='', coordinate=[], isGoal=False, size=0):
        """This class is used to represent a node. A node is an object on the shelf. Nodes are created when parsing json file. 

        Args:
            name (str, optional): [name of the object (tag)]. Defaults to ''.
            coordinate (list, optional): [coordinate of the object in the environment]. Defaults to [].
            isGoal (bool, optional): [set to True if the object is the one we want to reach]. Defaults to False.
            size (int, optional): [size of the object (radius size)]. Defaults to 0.
            child ([type = Node], optional): [child of the object see method for more]. Defaults to None.
            parent ([type = list(Node)], optional): [list of node referenced as parent of the actual node]. Defaults to None.
        """

        self.x, self.y, self.z = coordinate[0]['x'], coordinate[0]['y'], coordinate[0]['z']
        self.name = name
        self.size = size

        self.__child = []
        self.__parent = None

        self.__isGoal = isGoal

    def isRobotArmAccessible(self):
        """This method permits to know if the Node (object) is accessible to the robot arm. Which means the robot arm can grasp it without touching any object around 

        Returns:
            [boolean]: True if directly accessible
        """
        return self.__robotArmAccessible

    def setRobotArmAccessibility(self):
        """Update robot arm accessibility 
        """
        self.__robotArmAccessible = True

    def resetChild(self):
        self.__child = []

    def setChild(self, nodeChild):
        self.__child.append(nodeChild)

    def getChild(self):
        return self.__child

    def resetParent(self):
        self.__parent = None

    def setParent(self, parentChild):
        self.__parent = parentChild

    def getParent(self):
        return self.__parent

    def setGoal(self):
        self.__isGoal = True

    def isGoal(self):
        return self.__isGoal

    def getDistanceTo(self, datapoint):
        return sqrt(pow((datapoint[0]-self.x), 2) + pow((datapoint[1]-self.y), 2) + pow((datapoint[2]-self.z), 2))

    def getDistanceToNode(self, node):
        return sqrt(pow((node.x-self.x), 2) + pow((node.y-self.y), 2) + pow((node.z-self.z), 2))

    def __str__(self):
        """Equivalent of toString(). it permits to display all the info concerning the node
        """
        rep = "Object :\n"
        rep += "    name: " + self.name + "\n"
        rep += "    size: " + str(self.size) + "\n"

        rep += "    isGoal: " + str(self.__isGoal) + "\n\n"
        if(self.__child):
            for child in self.__child:
                rep += "    child: " + child.name + "\n"
        else:
            rep += "    child: " + str(self.__child) + "\n"

        if(self.__parent):
            rep += "    parent: " + self.__parent.name + "\n"
        else:
            rep += "    parent: " + str(self.__parent) + "\n"

        return rep
