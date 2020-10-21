from data.data import *
from utils.solver import *

import matplotlib.pyplot as plt


class shelf_object_solver():
    def __init__(self, shelf_size_x, shelf_size_y, precision):
        self.dataParser = Data("script/graph/data/objects.json")
        self.solver = Solver(shelf_size_x, shelf_size_y, precision)

        self.graph = []
        self.goal = None

    def createEnv(self):
        """
            Cette methode permet de creer le graph en fonction des proprietes de chaque objet definies dans le fichier .json
        """
        _, self.goal = self.solver.createGraph(self.graph)

        self.graph = self.solver.checkDirectConnectivity()
        self.graph, self.goal = self.solver.checkRobotArmGoalConnectivity([
                                                                          30, 50, 0])

        print("INFO - Env description")

        self.solver.createShelfGrid()

        for obj in self.graph:
            print(obj.__str__())

    def __solve(self):
        return self.solver.defineObjectToMove()

    def getData(self):
        """get data via rosservice
        """

        self.graph = self.dataParser.parseFile()

        print("INFO - Data Imported")

    def sendData(self):
        """send data via rosservice"""

        objectToMove = self.__solve()

        print("INFO - Data to send")
        # print(objectToMove)

    def visualize(self):
        """Cette fonction permet d'afficher le graph genere apres l'analyse de la position des objets et de leur accesibilite. 

            TODO add robot arm connectivity visualisation
        """

        x = []
        y = []
        s = []

        plt.gca().invert_yaxis()  # reverse y axis
        ax = plt.axes()

        for node in self.graph:
            x.append(node.x)
            y.append(node.y)
            s.append(node.size)

            plt.annotate(node.name, (node.x, node.y))

            if node.getParent():
                for parent in node.getParent():
                    ax.arrow(parent.x, parent.y, node.x - parent.x,
                             node.y - parent.y, head_width=0.5, head_length=1, fc='b', ec='b')

            if node.isDirectTrajectory():
                ax.arrow(node.x, node.y, self.goal.x - node.x,
                         self.goal.y - node.y, head_width=0.5, head_length=1, fc='r', ec='r')

        plt.scatter(x, y, color="k", s=s, label="Objects")
        plt.legend()

        plt.show()


if __name__ == "__main__":
    shelfdObjectSolver = shelf_object_solver(1000, 500, 1)

    shelfdObjectSolver.getData()

    shelfdObjectSolver.createEnv()

    shelfdObjectSolver.sendData()

    shelfdObjectSolver.visualize()
