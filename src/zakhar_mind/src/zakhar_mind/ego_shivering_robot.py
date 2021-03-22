from .mind_classes import EgoNode
from time import sleep


class Node(EgoNode):
    def main(self):
        while (1):
            self.to_will("shiver")
            sleep(5)


node = Node(name="EgoShiveringRobot")
