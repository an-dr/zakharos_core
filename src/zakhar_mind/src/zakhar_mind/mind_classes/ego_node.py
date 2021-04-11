import rospy
from time import sleep
from .exceptions import ReceivedAbort, ReceivedBusy, TerminateNode
from .generic_mind_node import GenericMindNode, node_types


class EgoNode(GenericMindNode):
    """
    This class has main function (shold be implemented) that starts with the node and works when it is not interrupted
    by other nodes
    """
    def __init__(self, name):
        super().__init__(name=name, node_type=node_types.EGO_NODE)
        self.terminate_flag = False

    def _main(self):
        """
        Main function supervisor
        """
        while (1):
            if self.terminate_flag:
                break
            try:
                self.main()
            except ReceivedAbort:
                rospy.logwarn("Abort")
                continue

    def main(self):
        raise NotImplementedError

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)
        self._init_communication()
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)
        self._main()

    def _to_will(self, symbol, modifier=()):
        rospy.loginfo("Ego is willing: %s:%s" % (symbol, str(modifier)))
        try:
            response = self.client_CommandConcept(self.node_type, symbol, str(modifier))
            res = response.result
            rospy.loginfo("Response for %s: %s" % (symbol, res))
            if res[:4] == "busy":
                raise ReceivedBusy
            elif res[:5] == "abort":
                raise ReceivedAbort
            elif res[:] == "error":
                rospy.logerr("Concept was executed with %s" % res)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            self.terminate_flag = True
            return str(e)

    def to_will(self, symbol, modifier=()):
        while 1:
            if self.terminate_flag:
                break
            try:
                return self._to_will(symbol, modifier)
            except ReceivedBusy:
                sleep(1)  # wait and retry
