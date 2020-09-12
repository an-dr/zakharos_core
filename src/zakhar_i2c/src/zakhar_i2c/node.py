from zakhar_pycore import zakhar__log as log
from zakhar_pycore import zakhar__i2c as i2c
import zakhar_common as com
from zakhar_msgs import msg, srv
import rospy
from time import sleep

class I2cNode:
    def __init__(self, name, log_level=log.INFO):
        self.name = name  # TODO make a hardcoded name
        self.l = log.get_logger(name, log_level=log_level)
        self.server = None

    def handle(self, req):
        self.l.debug("handle")
        try:
            if (req.node_cmd == "w"):
                self.l.debug("To write [ addr: 0x%x, reg:0x%x, val: 0x%x]" % (req.addr, req.reg, req.reg_value))
                i2c.i2c_write_byte_data(req.addr, req.reg, req.reg_value)
                return srv.I2cResponse(0x0AFF)
            elif (req.node_cmd == "r"):
                self.l.debug("To read [ addr: 0x%x, reg:0x%x]" % (req.addr, req.reg))
                read_byte = i2c.i2c_read_byte_from(req.addr, req.reg)
                return srv.I2cResponse(read_byte)
            else:
                self.l.error("Wrong I2C cmd. Variants: w, r")
                return srv.I2cResponse(0x0EFF)
        except IOError:
            if (req.node_cmd == "w"):
                self.l.error("Can't write")
            if (req.node_cmd == "r"):
                self.l.error("Can't read")
            sleep(.1)

    def start(self, log_level=None):
        if log_level is not None:
            self.l.setLevel(log_level)
        rospy.init_node(self.name, anonymous=False)
        self.server = com.get.server(name=self.name,
                                     service=srv.I2c,
                                     handle=self.handle,
                                     logger=self.l)
        self.l.info("[  DONE  ] Node \'%s\' is ready..." % self.name)
        rospy.spin()


node = I2cNode(name=com.names.NODE_I2C_SERVER, log_level=log.INFO)
