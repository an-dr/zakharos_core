import threading
import time
import rospy
import zakhar_common as com
from zakhar_msgs import srv, msg

import time

REG_CMD = 0
REG_MODE = 1

CMD_NONE = 0xFF
CMD_DONE = 0x00
CMD_STOP = 0xA0

CONFIG_MAX_RETRY = 16


class ZkI2cDevice(object):
    def __init__(self, dev_name, address, poll_dict={}):
        """
        [summary]

        Parameters
        ----------
        dev_name : str
        address : int
        req_poll_list : dict
            {
                (register) : (poll freq, hz),
                (register) : (poll freq, hz),
                ...
            }
            e.g.:
            {
                0x1: 10,
                0x2: 1
            }
        log_level : int, optional
        """

        self._name = dev_name
        self._address = address
        self.subscriber = None
        self.client = None
        self.client_lock = threading.Lock()
        self.publisher = None
        self._started = False
        self.poll_dict = poll_dict
        self.poll_threads = []

    def subscriber_callback(self, data):
        if ((data.target == self._name) or (data.target == 'all')):
            rospy.loginfo("Got [ %s -> %s ]: `%s`, %x, %x, %x, %x, `%s`" %
                          (rospy.get_caller_id(), data.target, data.msg, data.argumentA, data.argumentB, data.argumentC,
                           data.argumentD, data.argumentString))
            if (data.argumentString == 'w') or (data.argumentString == 'write'):
                self.write(reg=data.argumentA, val=data.argumentB)
            if (data.argumentString == 'r') or (data.argumentString == 'read'):
                self.read(reg=data.argumentA)

    def publish(self, dtype="", valA=0, valB=0, valC=0, valD=0, valString="", msg_note=""):
        to_send = msg.SensorData()
        to_send.type = dtype
        to_send.valA = valA
        to_send.valB = valB
        to_send.valC = valC
        to_send.valD = valD
        to_send.valString = valString
        to_send.message = msg_note
        self.publisher.publish(to_send)

    def poll_process(self, register, freq):
        while (1):
            try:
                d = self.read(reg=register)
            except Exception as e:
                d = None
            if d is None:
                d = -1
                continue
            self.publish(dtype=self._name, valA=register, valB=d, valC=0, valD=0, valString="", msg_note="A:reg, B:val")
            time.sleep(1 / freq)

    def _start_poll(self):
        for poll in self.poll_dict:
            t = threading.Thread(target=self.poll_process, args=(poll, self.poll_dict[poll]))
            t.daemon = True
            t.start()
            self.poll_threads.append(t)

    def run(self):
        rospy.logdebug("Node \'%s\' is starting..." % self._name)
        rospy.init_node(self._name, anonymous=False)
        ###
        rospy.loginfo("  Waiting for the service %s", com.names.NODE_I2C_SERVER)

        rospy.wait_for_service(com.names.NODE_I2C_SERVER)
        ###
        self.client = com.get.client(name=com.names.NODE_I2C_SERVER, service=srv.I2c)
        self.subscriber = com.get.subscriber(topic_name=com.names.TOPIC_DEVICECMD,
                                             data_class=msg.DeviceCmd,
                                             callback=self.subscriber_callback)
        self.publisher = com.get.publisher(topic_name=com.names.TOPIC_SENSORDATA, data_class=msg.SensorData)
        self._start_poll()
        ###
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self._name)

    def start(self):
        self.run()
        rospy.spin()

    def write(self, reg, val):
        rospy.logdebug("write(0x%x, 0x%x) to 0x%x" % (reg, val, self._address))
        if not self.client:
            raise rospy.ServiceException
        try:
            self.client_lock.acquire(blocking=True)
            resp = self.client("w", self._address, reg, val)
            self.client_lock.release()
            return resp.reg_value
        except rospy.ServiceException as e:
            if self.client_lock.locked:
                self.client_lock.release()
            rospy.logerr("Service call failed: %s" % e)

    def read(self, reg):
        rospy.logdebug("read(%x)" % (reg))
        if not self.client:
            raise rospy.ServiceException
        try:
            self.client_lock.acquire(blocking=True)
            resp = self.client("r", self._address, reg, 0)
            self.client_lock.release()
            return resp.reg_value
        except rospy.ServiceException as e:
            if self.client_lock.locked:
                self.client_lock.release()
            rospy.logerr("Service call failed: %s" % e)

    def mode(self):
        s = self.read(REG_MODE)
        return s

    def cmd(self, val):
        rospy.logdebug("Send a command :%s" % hex(val))
        self.write(REG_CMD, val)

    def cmd_stop(self):
        self.cmd(0xA0)
