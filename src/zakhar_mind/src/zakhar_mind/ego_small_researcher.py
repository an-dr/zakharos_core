from time import sleep
from .mind_classes import EgoNode


class Node(EgoNode):
    def main(self):
        while (1):
            for i in range(6):  # 300 ms of right
                self.to_will("move", ("forward"))
                sleep(.05)
            self.to_will("move", ("stop"))
            sleep(1)
            for i in range(6):  # 300 ms of right
                self.to_will("move", ("right"))
                sleep(.05)
            self.to_will("move", ("stop"))

            sleep(1)


node = Node(name="EgoSmallResearcher")
