class RobotArm():
    def __init__(self, x, y, z=0, name=" "):
        self.x, self.y, self.z = x, y, z

        self.name = "RobotArm - " + name

        self.__child = []
        self.__parent = None
        self.size = 100

    def setChild(self, nodeChild):
        self.__child.append(nodeChild)

    def getChild(self):
        return self.__child

    def getParent(self):
        return self.__parent

    def isGoal(self):
        return False

    def __str__(self):
        rep = "RobotArm :\n"

        rep += "    Pose (x,y,z): ("+str(self.x)+"," + \
            str(self.y)+","+str(self.z)+")\n"

        # if(self.__child):
        #     for child in self.__child:
        #         rep += "    child: " + child.name + "\n"
        # else:
        #     rep += "    child: " + str(self.__child) + "\n"

        return rep
