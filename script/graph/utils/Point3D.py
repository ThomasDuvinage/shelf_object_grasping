import copy as cp


class Point3D():
    def __init__(self, coordinate):
        if isinstance(coordinate, list):
            self.x = coordinate[0]
            self.y = coordinate[1]
            self.z = coordinate[2]
        else:
            self = cp.deepcopy(coordinate)
