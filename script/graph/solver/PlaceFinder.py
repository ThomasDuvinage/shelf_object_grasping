import math
import copy as cp

from utils.FreeZone import *


class PlaceFinder():
    """The goal of this class is to create a list of the potentiel place to move an object with a given size
    """

    def __init__(self, graph, shelf_size_x, shelf_size_y, precision, objectProximityRadius):
        self.shelf_size_x = shelf_size_x
        self.shelf_size_y = shelf_size_y
        self.objectRadiusProximity = objectProximityRadius
        self.precision = precision

        self.graph = graph

    def findPlace(self, diameter_object):
        notfind = False
        findplace = False
        box = [[0] * 2 for _ in range(4)]

        """ Liste des positions trouvé"""
        list_pos = []

        """ Centre d'une position trouvé """
        center = [1, 2]
        """fisrt point of the virtual box  """
        box[0][0] = 0
        box[0][1] = 0

        """second point of the virtual box  """
        box[1][0] = 0
        box[1][1] = diameter_object

        """third point of the virtual box  """
        box[2][0] = diameter_object
        box[2][1] = diameter_object

        """fourth point of the virtual box  """
        box[3][0] = diameter_object
        box[3][1] = 0

        while (notfind != True):

            if self.emptyArea(box):
                findplace = self.emptyArea(box)
            else:
                findplace = False

            if findplace:
                center[0] = (box[0][0] + box[3][0])/2
                center[1] = (box[0][1] + box[1][1])/2
                list_pos.append(cp.deepcopy(center))

            if (box[1][1]+self.precision < self.shelf_size_y):

                box[0][1] += self.precision
                box[1][1] += self.precision
                box[2][1] += self.precision
                box[3][1] += self.precision

            elif (box[3][0]+self.precision < self.shelf_size_x):

                box[0][1] = 0
                box[1][1] = diameter_object
                box[2][1] = diameter_object
                box[3][1] = 0

                box[0][0] += self.precision
                box[1][0] += self.precision
                box[2][0] += self.precision
                box[3][0] += self.precision

            else:
                notfind = True

        newList = []
        for i, point in enumerate(list_pos):
            newFreeZone = FreeZone(
                "FreeZone" + str(i), [point[0], point[1], 0], diameter_object)
            newList.append(newFreeZone)

        return newList

    def emptyArea(self, box):
        empty_area = True
        for point in self.graph:
            if (point.x < box[3][0] and point.x > box[0][0] and point.y < box[1][1] and point.y > box[0][1]):
                empty_area = False

            for j in range(1, int(point.size/10)):
                for i in range(0, 10):
                    angle = math.radians(i*36)
                    rayon = j * 10
                    point_x = rayon * math.cos(angle) + point.x
                    point_y = rayon * math.sin(angle) + point.y
                    if (point_x < box[3][0] and point_x > box[0][0] and point_y < box[1][1] and point_y > box[0][1]):
                        empty_area = False

        return empty_area
