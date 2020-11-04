from PySide2.QtWidgets import *
from PySide2.QtGui import *
from PySide2.QtCore import *
import sys

from Node import Node


class ShelfController(QMainWindow):
    def __init__(self, x_size, y_size):
        super(ShelfController, self).__init__()
        self.shelfRect = QRectF(0,0,x_size,y_size)

        self.setWindowTitle("Shelf Grasping Controller")

        self.create_ui()
        self.show()

    def create_ui(self):
        widget = QWidget()

        #### Button Node Events ####
        buttonAdd = QPushButton("Add Object", self)
        buttonAdd.clicked.connect(self.addNode)

        buttonClear = QPushButton("Clear shelf", self)
        buttonClear.clicked.connect(self.clearShelf)
        ############################

        #### Node Info Editor #####
        checkboxIsGoal = QCheckBox(self)

        sizeEditor = QLineEdit()
        sizeEditor.setValidator(QDoubleValidator(1.00,999.99,2))

        xEditor = QLineEdit()
        xEditor.setValidator(QDoubleValidator(0.00,self.shelfRect.width(),2))
        xEditor.setMinimumWidth(50)

        yEditor = QLineEdit()
        yEditor.setValidator(QDoubleValidator(0.00,self.shelfRect.height(),2))
        ############################

        self.scene = QGraphicsScene(self)
        self.scene.setSceneRect(self.shelfRect)
        self.view = QGraphicsView(self.scene, self)

        groupBox = QGroupBox("Node Infos")
        nodeInfosBox = QFormLayout()
        nodeInfosBox.addRow("Is goal :", checkboxIsGoal)
        nodeInfosBox.addRow("Size :", sizeEditor)
        nodeInfosBox.addRow("X pose :", xEditor)
        nodeInfosBox.addRow("Y pose :", yEditor)

        groupBox.setLayout(nodeInfosBox)

        hbox = QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(self.view)
        hbox.addWidget(groupBox)

        buttonHBox = QHBoxLayout()
        buttonHBox.addStretch(1)
        buttonHBox.addWidget(buttonAdd)
        buttonHBox.addWidget(buttonClear)

        masterVBox = QVBoxLayout()
        masterVBox.addStretch(1)
        masterVBox.addLayout(hbox)
        masterVBox.addLayout(buttonHBox)

        widget.setLayout(masterVBox)

        self.setCentralWidget(widget)

    def addNode(self):
        newNode = Node(self.scene,10,10,100)
        newNode.addToScene()

    def clearShelf(self):
        self.scene.clear()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ShelfController(700, 450)
    sys.exit(app.exec_())
