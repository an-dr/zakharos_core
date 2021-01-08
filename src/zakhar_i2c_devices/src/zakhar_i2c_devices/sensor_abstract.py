import rospy
from time import sleep
import threading
from zakhar_i2c import ZkI2cDevice
from typing import Any


class ZkSensor():
    def __init__(self, name, sensor_platform):
        self.name = name
        self.sensor_platform = sensor_platform  # type:ZkI2cDevice
        self.termination = False
        self.poll_thread = None  # type: Any[threading.Thread, None]
        self.poll_freq = 0

    def publish(self, dtype="", valA=0, valB=0, valC=0, valD=0, valString="", msg_note=""):
        self.sensor_platform.publish(dtype=dtype,
                                     valA=valA,
                                     valB=valB,
                                     valC=valC,
                                     valD=valD,
                                     valString=valString,
                                     msg_note=msg_note)

    def __poll_loop(self):
        rospy.loginfo("Polling start")
        while 1:
            if self.termination:
                break
            self.poll_once()
            sleep(1 / self.poll_freq)
        self.poll_freq = 0
        rospy.loginfo("Polling end")

    def start(self, freq_hz=10):
        """
        Parameters
        ----------
        freq : int, optional
            Hz, by default 10
        """
        self.termination = False
        self.poll_freq = freq_hz
        self.start_setup()
        self.poll_thread = threading.Thread(name='[%s]polling' % self.name, target=self.__poll_loop)
        self.poll_thread.setDaemon(True)
        self.poll_thread.start()

    def stop(self):
        self.termination = True

    def poll_once(self):
        raise NotImplementedError()

    def start_setup(self):
        raise NotImplementedError()
