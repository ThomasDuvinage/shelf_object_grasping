from utils.PlaceFinder import *
import numpy as np
import sys


np.set_printoptions(threshold=sys.maxsize)


class Solver(PlaceFinder):
    def __init__(self, shelf_size_x, shelf_size_y, precision):
        """ This class permits to find the right strategy to reach the goal object without touching any other object.

        Args:
            shelf_size_x (int): x size of the shelf in mm
            shelf_size_y (int): y size of the shelf in mm
            precision (int): number of mm per pixel
        """
        self.__graph = []
        self.goal = None

        self.objectRadiusProximity = 5  # defined depending on the arm's size

        self.precision = precision

        # initialize a grid representing the shelf
        self.__shelf = np.zeros(
            (int(shelf_size_y/precision), int(shelf_size_x/precision)))

        super().__init__(self.__shelf, precision)

    def createShelfGrid(self):
        """This method permits to create the shelf grid which will be used to determine a future pose for an object and check if an object is graspable or not to finish building the graf
        """

        for objects in self.__graph:

            y_sizeMin = int((objects.y / self.precision) -
                            objects.size/(2 * self.precision))
            y_sizeMax = int((objects.y / self.precision) +
                            objects.size/(2 * self.precision))

            x_sizeMin = int((objects.x / self.precision) -
                            objects.size/(2 * self.precision))
            x_sizeMax = int((objects.x / self.precision) +
                            objects.size/(2 * self.precision))

            for i in range(y_sizeMin, y_sizeMax):
                for u in range(x_sizeMin, x_sizeMax):
                    self.__shelf[i][u] = 1

            print(self.__shelf)

    def createGraph(self, node_array):
        """ Cette methode permet de generer le graph en fonction des objets dans l'espace

        Un noeud ne peut avoir qu'un fils par contre il peut avoir plusieurs parents.
        """
        for node in node_array:
            if not node.isGoal():
                child = node.getClosestNode(node_array)

                child.setParent(node)
                node.setChild(child)
            else:
                self.goal = node

        self.__graph = node_array

        return self.__graph, self.goal

    def checkRobotArmGoalConnectivity(self, robotArmPose):
        """INFO : Cette fonction va changer car on recuperera la position du bras grace a ros

        TODO add shelf boundary size
        TODO update !!

        Args:
            robotArmPose ([x,y,z]): robot arm pose in the env
        """
        for node in self.__graph:
            objects_array = self.__getObjectInBand(node)

            if objects_array:
                close_node = self.__getClosestObjectsFromRobotArm(
                    objects_array, robotArmPose)

                if close_node.isAccessible(self.objectRadiusProximity):
                    node.setRobotArmAccessibility()

        return self.__graph, self.goal

    def checkDirectConnectivity(self):
        """Cette fonction permet de verifier si un node est peut etre relie directement au node goal

        TODO update !!
        """

        for node in self.__graph:
            if not node.isGoal():
                objects_array = self.__getObjectInBand(node)

                if objects_array:
                    close_node = self.__getClosestObjectsFromNode(
                        objects_array)

                    if close_node.isAccessible(self.objectRadiusProximity):
                        node.setDirectTrajecory()
        return self.__graph

    def defineObjectToMove(self):
        """Cette methode definit la liste des objets a bouger pour atteindre l'object goal.
            En verifiant auparavant si l'objet goal ne peut pas etre atteint directement.
        """
        pass

    def __getObjectInBand(self, node):
        """Le concept de cette methode est de creer une bande virtuelle sur l'axe des x en fonction d'un node donne + rayon d'accesibilite qui varie en fonction de la taille du bras. Cette methode retourne ensuite tous les nodes qui sont dans cette bande.

        Returns:
            [list(Node)]: liste des nodes dans la bandes
        """
        objects = []
        for obj in self.__graph:
            if obj.x < node.x + self.objectRadiusProximity:
                if obj.x > node.x - self.objectRadiusProximity:
                    objects.append(obj)
        return objects

    def __getClosestObjectsFromRobotArm(self, array, robotArmPose):
        closest_node = None
        min_dist = 100000

        for node in array:
            if(min_dist > node.getDistanceTo(robotArmPose)):
                closest_node = node
                min_dist = node.getDistanceTo(robotArmPose)

        return closest_node

    def __getClosestObjectsFromNode(self, array):
        closest_node = None
        min_dist = 100000

        for obj in array:
            if not obj.isGoal():
                if(min_dist > self.goal.getDistanceToNode(obj)):
                    closest_node = obj
                    min_dist = self.goal.getDistanceToNode(obj)

        return closest_node
