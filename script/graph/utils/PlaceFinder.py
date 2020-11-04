class PlaceFinder():
    """The goal of this class is to find the right place to move an object with a given size
    """

    def __init__(self, _shelf, precision):
        """ This class permits to find the best place for an objet. This function is used after finding objects that need to be moved to access the goal object. 

        Args:
            _shelf (numpy array): reference of the np array represented the shelf grid created in the Solver constructor
            precision (int): precision of the grid (mm per pixel)
        """
        self.shelf = _shelf

    def findPlaceForObject(self, obj):
        """L'objectif de cette fonction est de trouver la bonne place pour un objet dans l'etagere. 

        Il faut :
            - trouver un endroit vide
            - accesible part le bras de robot

        Args:
            obj (Nodes): Objet Nodes representant l'objet que l'on souhaite deplace.
        """
        pass
