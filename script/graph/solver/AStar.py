from math import inf


class AStar():
    def __init__(self):
        pass

    def solve(self, startingNode, goalNode):
        closestList = []
        openList = []
        openList.append(startingNode)

        while openList:
            state = self.getMinimumFunctionNode(openList)
            closestList.append(state.name)

            if state.isGoal():
                return state

            for child in state.getSucessors():
                if child.name in closestList or self.checkChildInOpenList(openList, child):
                    child.updateCostG(state)
                    child.updateFunctionValue()
                    child.setParent(state)
                    openList.append(child)

    def getMinimumFunctionNode(self, nodeList):
        minimum = inf
        best_node = None

        for node in nodeList:
            if node.getFunctionValue() < minimum:
                best_node = node
                minimum = node.getFunctionValue()

        return best_node

    def checkChildInOpenList(self, openList, child):
        for node in openList:
            if child.name == node.name:
                if child.getFunctionValue() < node.getFunctionValue():
                    return True

        return False
