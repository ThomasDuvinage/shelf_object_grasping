from utils.Zone import *


class Node(Zone):

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

        super().__init__(name, [coordinate[0]['x'],
                                coordinate[0]['y'], coordinate[0]['z']], size)

        self.__isGoal = isGoal

    def setGoal(self):
        self.__isGoal = True

    def isGoal(self):
        return self.__isGoal

    def __str__(self):
        """Equivalent of toString(). it permits to display all the info concerning the node
        """
        rep = "Object :\n"
        rep += "    name: " + self.name + "\n"
        rep += "    size: " + str(self.size) + "\n"

        rep += "    isGoal: " + str(self.__isGoal) + "\n\n"
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
