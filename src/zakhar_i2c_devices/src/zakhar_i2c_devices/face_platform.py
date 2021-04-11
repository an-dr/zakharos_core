import rospy
from zakhar_i2c import ZkI2cDevice
from zakhar_pycore import dev as zdev


class I2cFacePlatform(ZkI2cDevice):
    def __init__(self, dev_name, address, poll_dict={}):
        ZkI2cDevice.__init__(self, dev_name, address, poll_dict)

    def start(self, log_level=None):
        super(I2cFacePlatform, self).run()
        self.write(0, zdev.face_module.cmd.CALM)
        rospy.spin()


face_platform = I2cFacePlatform(dev_name=zdev.face_module.name, address=zdev.face_module.addr)
