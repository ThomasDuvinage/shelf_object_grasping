# This Python file uses the following encoding: utf-8
from PySide2.QtWidgets import *
from PySide2.QtGui import *
from PySide2.QtCore import *


class Node(QGraphicsEllipseItem):
    def __init__(self, scene = None, x=0, y=0, size=0):
        super(Node,self).__init__()
        self._scene = scene

        self.x,self.y, self.w, self.h = x, y, size,size

        self._brushColor = QBrush(Qt.blue)
        self._penColor = QPen(Qt.black)

        self.setBrush(self._brushColor)
        self.setPen(self._penColor)

        self._isGoal = False

    def addToScene(self, x, y, size):
        if self._isGoal:
            self._brushColor = QBrush(Qt.red)

        self.setBrush(self._brushColor)
        self.setPen(self._penColor)
        self.x = x
        self.y = y

        self.w = size
        self.h = size

        self.node = self._scene.addEllipse(self.x, self.y, self.w, self.h, self.pen(), self.brush())
        self.node.setFlag(QtGraphicsItem.ItemIsMovable)

    def addToScene(self):
        self.node = self._scene.addEllipse(self.x, self.y, self.w, self.h, self.pen(), self.brush())
        self.node.setFlag(QGraphicsItem.ItemIsMovable)

    def mouseReleaseEvent(self, event):
        print("hello")

        return QtGui.QGraphicsEllipseItem.mouseReleaseEvent(self, event)

    def hoverMoveEvent(self, event):
        print("coucou")
