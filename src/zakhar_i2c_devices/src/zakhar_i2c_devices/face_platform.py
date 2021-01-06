import rospy
from os.path import basename, splitext
from zakhar_i2c import ZkI2cDevice

ADDR_FACE = 0x2c
CMD_CALM = 0x30
CMD_BLINK = 0x31
CMD_ANGRY = 0x32
CMD_HAPPY = 0x33
CMD_SAD = 0x34

class I2cFacePlatform(ZkI2cDevice):
    def __init__(self, dev_name, address, poll_dict={}):
        ZkI2cDevice.__init__(self,dev_name,address,poll_dict)

    def start(self, log_level=None):
        super(I2cFacePlatform, self).run()
        self.write(0, CMD_CALM)
        rospy.spin()


DEV_NAME = splitext(basename(__file__))[0]  # filename without extension
face_platform = I2cFacePlatform(dev_name=DEV_NAME, address=ADDR_FACE)


