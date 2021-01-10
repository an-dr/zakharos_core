import rospy
from os.path import basename, splitext
from zakhar_pycore.constants import ADDR
from zakhar_i2c import ZkI2cDevice
from .sensor_photo_resistor import ZkSensorPhotoResistor
from .sensor_sound_distance import ZkSensorSoundDistance


class I2cSensorPlatform(ZkI2cDevice):
    def __init__(self, dev_name, address, poll_dict={}):
        ZkI2cDevice.__init__(self, dev_name, address, poll_dict)
        self.sensor_photo_res = ZkSensorPhotoResistor(sensor_platform=self,
                                                      corr_window_ms=500,
                                                      pattern=[0, 0, 0, 255, 0, 0, 0],
                                                      threshold=0.6)
        self.sensor_distance = ZkSensorSoundDistance(sensor_platform=self)

    def start(self):
        super(I2cSensorPlatform, self).run()
        self.sensor_photo_res.start(10)
        self.sensor_distance.start(10)
        rospy.spin()


DEV_NAME = splitext(basename(__file__))[0]  # filename without extension
sensor_platform = I2cSensorPlatform(dev_name=DEV_NAME, address=ADDR.I2C.SENSORS)
