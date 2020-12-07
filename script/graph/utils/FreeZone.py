import copy as cp

from utils.Zone import *


class FreeZone(Zone):
    def __init__(self, name, coordinate, size):
        super().__init__(name, coordinate, size)

    def moveParent(self):
        self._parent.x = self.x
        self._parent.y = self.y
        self._parent.z = self.z

        print("id before : ", id(self._parent))

        self._parent = cp.deepcopy(self._parent)

        print("id after : ", id(self._parent))
