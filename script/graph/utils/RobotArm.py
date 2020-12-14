from utils.Zone import *


class RobotArm(Zone):
    def __init__(self, x, y, z=0, name=" "):
        super().__init__("RobotArm - " + name, [x, y, z], 150)

    def __str__(self):
        rep = "RobotArm :\n"

        rep += "    Pose (x,y,z): ("+str(self.x)+"," + \
            str(self.y)+","+str(self.z)+")\n"

        return rep
