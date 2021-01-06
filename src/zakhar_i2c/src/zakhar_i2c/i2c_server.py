from zakhar_pycore import i2c
import zakhar_common as com
from zakhar_msgs import msg, srv
import rospy
from time import sleep
from os.path import basename, splitext

class I2cServerNode:
    def __init__(self):
        self.name = splitext(basename(__file__))[0]
        self.server = None

    def handle(self, req):
        rospy.logdebug("handle")
        try:
            if (req.node_cmd == "w"):
                rospy.logdebug("To write [ addr: 0x%x, reg:0x%x, val: 0x%x]" % (req.addr, req.reg, req.reg_value))
                i2c.write(req.addr, req.reg, req.reg_value)
                return srv.I2cResponse(0x0AFF)
            elif (req.node_cmd == "r"):
                rospy.logdebug("To read [ addr: 0x%x, reg:0x%x]" % (req.addr, req.reg))
                read_byte = i2c.read(req.addr, req.reg)
                return srv.I2cResponse(read_byte)
            else:
                rospy.logerr("Wrong I2C cmd. Variants: w, r")
                return srv.I2cResponse(0x0EFF)
        except IOError:
            if (req.node_cmd == "w"):
                rospy.logerr("Can't write")
            if (req.node_cmd == "r"):
                rospy.logerr("Can't read")
            sleep(.1)

    def start(self):

        rospy.init_node(self.name, anonymous=False)
        self.server = com.get.server(name=self.name,
                                     service=srv.I2c,
                                     handle=self.handle)
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)
        rospy.spin()


def start():
    node = I2cServerNode()
    node.start()
