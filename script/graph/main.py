from data.data import *
from utils.Nodes import *
from utils.solver import *

import matplotlib.pyplot as plt

import logging as LOGGER
from random import randint


class shelf_object_solver():
    def __init__(self, shelf_size_x, shelf_size_y, precision, verbose=True):
        self.dataParser = Data("script/graph/data/objects.json")
        self.solver = Solver(shelf_size_x, shelf_size_y, precision)

        self.x_boundary = shelf_size_x
        self.y_boundary = shelf_size_y

        self.graph = []
        self.goal = None

        # Warning may not be true
        self.__grasper = RobotArm(shelf_size_x / 2, shelf_size_y)

        self.__verbose = verbose

    def createEnv(self):
        """
            Cette methode permet de creer le graph en fonction des proprietes de chaque objet definies dans le fichier .json
        """
        self.goal = self.solver.createGraph(self.graph)

        self.graph = self.solver.checkDirectConnectivity()
        self.graph = self.solver.checkDirectConnectivity(
            robotArm=self.__grasper)

        LOGGER.info("Env description")

        if self.__verbose:
            for obj in self.graph:
                print(obj.__str__())

    def __solve(self):
        return self.solver.defineObjectToMove()

    def getData(self, randomInit=False, objectNumber=0):
        """get data via rosservice
        """

        if not randomInit:
            self.graph = self.dataParser.parseFile()

        else:

            x = randint(0, self.x_boundary)
            y = randint(0, self.y_boundary/4)
            objSize = randint(50, 100)  # Can be modified

            newObj = Node(
                "ObjectGoal", [{'x': x, 'y': y, 'z': 0}], isGoal=True, size=objSize)
            self.graph.append(newObj)

            for i in range(objectNumber - 1):
                x = randint(20, self.x_boundary - 20)
                y = randint(20, self.y_boundary - 20)
                objSize = randint(50, 100)  # Can be modified

                newObj = Node(
                    "Object"+str(i), [{'x': x, 'y': y, 'z': 0}], isGoal=False, size=objSize)
                self.graph.append(newObj)

        LOGGER.info("Data Imported")

    def sendData(self):
        """send data via rosservice"""

        objectToMove = self.__solve()

        LOGGER.info("Data to send")

        # print(objectToMove)

    def visualize(self):
        """Cette fonction permet d'afficher le graph genere apres l'analyse de la position des objets et de leur accesibilite.

            TODO add robot arm connectivity visualisation
        """

        x = []
        y = []
        s = []

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, constrained_layout=True)
        fig.suptitle("Links between Nodes and robot arm")

        ax1.set_xlim([0, self.x_boundary])
        ax1.set_ylim([0, self.y_boundary])

        ax2.set_xlim([0, self.x_boundary])
        ax2.set_ylim([0, self.y_boundary])

        ax3.set_xlim([0, self.x_boundary])
        ax3.set_ylim([0, self.y_boundary])

        ax1.invert_yaxis()  # reverse y axis
        ax2.invert_yaxis()
        ax3.invert_yaxis()

        ax1.set_title("Nodes proximity links")
        ax2.set_title("Direct trajectories to goal object")
        ax3.set_title("Robot arm accesibility graph")

        for node in self.graph:
            x.append(node.x)
            y.append(node.y)
            s.append(node.size)

            ax1.annotate(node.name, (node.x, node.y))
            ax2.annotate(node.name, (node.x, node.y))
            ax3.annotate(node.name, (node.x, node.y))

            if node.getParent():
                for parent in node.getParent():
                    ax1.arrow(parent.x, parent.y, node.x - parent.x,
                              node.y - parent.y, head_width=0.1, head_length=1, fc='b', ec='b')

            if node.isDirectTrajectory():
                ax2.arrow(node.x, node.y, self.goal.x - node.x,
                          self.goal.y - node.y, head_width=0.1, head_length=1, fc='r', ec='r')

            if node.isRobotArmAccessible():
                ax3.arrow(self.__grasper.x, self.__grasper.y, node.x - self.__grasper.x,
                          node.y - self.__grasper.y, head_width=0.1, head_length=1, fc='g', ec='g')

        ax1.scatter(x, y, color="k", s=node.size*2, label="Objects")
        ax2.scatter(x, y, color="k", s=node.size*2)
        ax3.scatter(x, y, color="k", s=node.size*2)
        fig.legend()

        plt.show()


if __name__ == "__main__":
    shelfdObjectSolver = shelf_object_solver(800, 400, 1, verbose=False)

    shelfdObjectSolver.getData(randomInit=True, objectNumber=10)

    shelfdObjectSolver.createEnv()

    shelfdObjectSolver.sendData()

    shelfdObjectSolver.visualize()
