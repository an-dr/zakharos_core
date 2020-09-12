import rospy

import zakhar_common as com
import zakhar_log as log
import zakhar_i2c_devices as dev


if __name__ == "__main__":
    dev.sensor_platform.start()
    rospy.spin()