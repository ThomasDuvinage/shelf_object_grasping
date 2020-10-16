import json
import random
from utils.Nodes import *


class Data():
    def __init__(self, filename=""):
        self.filename = filename

    def parseFile(self):
        """This method parse the json file describing the grid into a python array

        Args:
            grid_array ([Node]): array of Nodes representing the grid
        """
        grid_array = []

        with open(self.filename) as json_file:
            points = json.load(json_file)

            for point in points:
                grid_array.append(
                    Node(point["name"], point["coordinate"], point["isGoal"], point["size"]))

        return grid_array
