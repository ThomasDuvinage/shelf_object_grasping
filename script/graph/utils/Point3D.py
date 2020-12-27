import copy as cp
import numpy as np

from math import sqrt, pow, degrees


class Point3D():
    def __init__(self, coordinate):
        if isinstance(coordinate, list):
            self.x = coordinate[0]
            self.y = coordinate[1]
            self.z = coordinate[2]
        else:
            self = cp.deepcopy(coordinate)

    def computeAngle(self, parent):
        vectorPC = [self.x - parent.x, self.y - parent.y]
        vectorPy = [0, 1 - parent.y]

        unitVecPC = vectorPC / np.linalg.norm(vectorPC)
        unitVecPy = vectorPy / np.linalg.norm(vectorPy)

        dot_product = np.dot(unitVecPC, unitVecPy)

        angle = np.arccos(dot_product) * 1.5

        angle = degrees(angle)

        return angle
