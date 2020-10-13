from utils.PlaceFinder import *

class Solver(PlaceFinder):
    def __init__(self):
        super().__init__()
        self.__graph = []

    def createGraph(self, node_array):
        """ Cette methode permet de generer le graph en fonction des objets dans l'espace 
        """
        goalNode = node_array[0]

        for node in node_array[1:]:
            child = node.getClosestNode(node_array[1:])

            child.setParent(node)
            node.setChild(child)

        self.__graph = node_array

    
    def __checkRobotArmGoalConnectivity(self, robotArmPose):
        """TODO cette fonctionne va changer car on recuperera la position du bras grace a ros 

        Args:
            robotArmPose ([type]): robot arm pose in the env
        """
        pass


    def __checkDirectConnectivity(self):
        """Cette fonction permet de verifier si un node est peut etre relie directement au node goal"""
        pass


    def defineObjectToMove(self):
        """Cette methode definit la liste des objets a bouger pour atteindre l'object goal.
            En verifiant auparavant si l'objet goal ne peut pas etre atteint directement.
        """
        pass
