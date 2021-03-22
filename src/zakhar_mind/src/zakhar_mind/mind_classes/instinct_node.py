from .generic_mind_node import GenericMindNode, node_types


class InstinctNode(GenericMindNode):
    def __init__(self, name):
        super().__init__(name=name, node_type=node_types.INSTINCT_NODE)
