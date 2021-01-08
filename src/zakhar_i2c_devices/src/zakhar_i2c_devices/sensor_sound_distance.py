import rospy
from .sensor_abstract import ZkSensor

CONFIG_PRINT_VAL = True
MSG_SENSOR_VALUE = "distances"

REG_DIST_L = 0x02
REG_DIST_C = 0x03
REG_DIST_R = 0x04


class ZkSensorSoundDistance(ZkSensor):
    def __init__(self, sensor_platform):
        ZkSensor.__init__(self, name="sound_dist", sensor_platform=sensor_platform)
        self.last_val = [0xffff, 0xffff, 0xffff]

    def read_distance(self):
        try:
            val_l = self.sensor_platform.read(REG_DIST_L)
            val_c = self.sensor_platform.read(REG_DIST_C)
            val_r = self.sensor_platform.read(REG_DIST_R)
        except rospy.ServiceException:
            return [0xffff, 0xffff, 0xffff]
        return [val_l, val_c, val_r]

    def start_setup(self):
        pass

    def poll_once(self):
        # read
        self.last_val = self.read_distance()
        if CONFIG_PRINT_VAL:
            rospy.loginfo("Distances : " + str(self.last_val))
        # publishing
        self.publish(dtype=MSG_SENSOR_VALUE, valA=self.last_val[0], valB=self.last_val[1], valC=self.last_val[2])
