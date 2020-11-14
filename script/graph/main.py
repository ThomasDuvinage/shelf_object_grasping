from data.data import *
from utils.Nodes import *
from utils.solver import *

import matplotlib.pyplot as plt

import logging as LOGGER
from random import randint


class shelf_object_solver():
    def __init__(self, shelf_size_x, shelf_size_y, precision, randomENV=False, verbose=True):
        self.dataParser = Data("script/graph/data/objects.json")

        self.x_boundary = shelf_size_x
        self.y_boundary = shelf_size_y

        self.graph = []

        self.getData(randomInit=randomENV, objectNumber=10)
        self.goal = self.__getGoal()

        self.solver = Solver(shelf_size_x, shelf_size_y,
                             precision, self.graph, self.goal)

        # Warning may not be true
        self.__grasper = RobotArm(shelf_size_x / 2, shelf_size_y)

        self.__verbose = verbose

    def __solve(self):

        return self.solver.defineObjectToMove(self.__grasper, "BFS", occurence_test=True)

    def __getGoal(self):
        for obj in self.graph:
            if obj.isGoal():
                return obj
        return None

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

        objectsToMove, iterations = self.__solve()

        LOGGER.info("Data to send")

        nb_objects_move = 0

        for obj in objectsToMove:
            nb_objects_move += 1
            print(obj.__str__())

        print("Solve in ", iterations, "iterations")
        print(nb_objects_move, "have to be moved to reach goal Object")

        return objectsToMove

    def visualize(self, solution=None):
        """Cette fonction permet d'afficher le graph genere apres l'analyse de la position des objets et de leur accesibilite.

            TODO add robot arm connectivity visualisation
        """

        x_graph = []
        y_graph = []
        s_graph = []

        x_solution = []
        y_solution = []
        s_solution = []

        fig, (ax1, ax2) = plt.subplots(2, 1, constrained_layout=True)
        fig.suptitle("Links between Nodes and robot arm")

        ax1.set_xlim([0, self.x_boundary])
        ax1.set_ylim([0, self.y_boundary])

        ax2.set_xlim([0, self.x_boundary])
        ax2.set_ylim([0, self.y_boundary])

        ax1.invert_yaxis()  # reverse y axis
        ax2.invert_yaxis()

        ax1.set_title("Nodes connections")
        ax2.set_title("Solution")

        for node in self.graph:
            x_graph.append(node.x)
            y_graph.append(node.y)
            s_graph.append(node.size)

            ax1.annotate(node.name, (node.x, node.y))

            if node.getChild():
                for child in node.getChild():
                    ax1.arrow(child.x, child.y, node.x - child.x,
                              node.y - child.y, head_width=0.1, head_length=1, fc='b', ec='b')

        if solution:
            for node in solution:
                ax2.annotate(node.name, (node.x, node.y))

                x_solution.append(node.x)
                y_solution.append(node.y)
                s_solution.append(node.size)

            for i in range(len(solution) - 1):
                ax2.arrow(solution[i].x, solution[i].y, solution[i+1].x - solution[i].x,
                          solution[i+1].y - solution[i].y, head_width=0.1, head_length=1, fc='b', ec='b')

        ax1.scatter(x_graph, y_graph, color="k",
                    s=node.size*2, label="Objects")
        ax2.scatter(x_solution, y_solution, color="k",
                    s=node.size*2)

        fig.legend()

        plt.show()


if __name__ == "__main__":
    shelfdObjectSolver = shelf_object_solver(
        2000, 700, 1, randomENV=True, verbose=False)

    solution = shelfdObjectSolver.sendData()

    shelfdObjectSolver.visualize(solution=solution)
