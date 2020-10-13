from data.data import *
from utils.solver import *

class shelf_object_solver():
    def __init__(self):
        self.dataParser = Data("data/objects.json")
        self.solver =  Solver()

        self.objects = []

    def createEnv(self):
        self.solver.createGraph(self.objects)

        print("INFO - Env description")

        for obj in self.objects:
            print(obj.__str__())

    def __solve(self):
        return self.solver.defineObjectToMove()

    def getData(self):
        """get data via rosservice

            !!! Important !!! the goal object has to be the first in the transmitted message
        """

        self.objects = self.dataParser.parseFile()

        print("INFO - Data Imported")

    def sendData(self):
        """send data via rosservice"""

        objectToMove = self.__solve()

        print("INFO - Data to send")
        #print(objectToMove)


if __name__ == "__main__":
    shelfdObjectSolver = shelf_object_solver()

    shelfdObjectSolver.getData()

    shelfdObjectSolver.createEnv()

    shelfdObjectSolver.sendData()