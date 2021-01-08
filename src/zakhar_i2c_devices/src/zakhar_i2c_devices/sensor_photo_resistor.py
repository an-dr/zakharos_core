import rospy
from .sensor_abstract import ZkSensor
from zakhar_pycore.helpers.format import list2strf
from collections import deque
import numpy as np
from PIL import Image
from time import time
from typing import Any

REG_VAL_LO = 0x06
REG_VAL_HI = 0x05

CONFIG_PRINT_HALF_CORR = True
CONFIG_PRINT_CORR = False
CONFIG_PRINT_WINDOW = False
CONFIG_PRINT_LIGHT = False

SHADOW_CONTRAST_PRST = 0.05
MINIMAL_TRIG_PERIOD_SEC = 3

MIN_LIGHT = 0
MAX_LIGHT = 0xffff
# CONFIG_SHADOW_CONTRAST_PRST = 0.5

MSG_SENSOR_VALUE = "light"
MSG_TRIG = "bird"

POLL_PERIOD = 0.01  # sec
WINDOWS_SIZE_SEC = 0.5  # sec
WINDOW_SIZE_ELEMENTS = int(WINDOWS_SIZE_SEC / POLL_PERIOD)


# TODO move to zakhar_pycore
def round_list(in_list, num):
    new_l = []
    for i in in_list:
        new_l.append(round(i, num))
    return new_l


# TODO move to zakhar_pycore
def calc_max_deviation(in_list):
    lmax = float(max(in_list))
    lmin = float(min(in_list))
    delta = lmax - lmin
    if not delta:
        return 0
    return delta / lmax


class ZkSensorPhotoResistor(ZkSensor):
    def __init__(self, sensor_platform, corr_window_ms, pattern, threshold):
        ZkSensor.__init__(self, name="photoresistor", sensor_platform=sensor_platform)
        self.mon_windows_sz_ms = corr_window_ms
        self.mon_window_sz_elements = None
        self.mon_window = None  # type: Any[deque, None]
        self.corr_pattern_raw = pattern
        self.corr_pattern = None
        self.threshold = threshold  # type: Any[int]
        self.last_light = 0
        self.last_corr_coef = 0
        self.last_trig_time = -1

    def read_light(self):
        try:
            lo = self.sensor_platform.read(REG_VAL_LO)
            hi = self.sensor_platform.read(REG_VAL_HI)
        except rospy.ServiceException:
            return 0xffff
        if lo is None or hi is None:
            return 0xffff
        val = (hi << 8) | lo
        return val

    def write_to_window(self, value, keep_extreme_vals=False):
        if self.mon_window is not None:
            if (self.last_light != MIN_LIGHT) and (self.last_light != MAX_LIGHT):  # check if value is not extremal
                self.mon_window.append(value)
            if keep_extreme_vals:  # extremeal value writing depends on the paramter
                self.mon_window.append(value)

    def init_window(self, size_ms):
        """
        Parameters
        ----------
        size_ms : int

        Returns
        -------
        int
            elements in the window
        """
        win_el_num = int((1000 * self.poll_freq) / size_ms)
        rospy.loginfo("Windows inited, size is %d" % win_el_num)
        self.mon_window = deque([0] * win_el_num, maxlen=win_el_num)
        return len(self.mon_window)

    def deinit_window(self):
        self.mon_window = None

    def trig_on_threshold(self, corr_coef):
        # protection against frequent trigging, then comparing with the threshold
        if (time() > self.last_trig_time + MINIMAL_TRIG_PERIOD_SEC) and (corr_coef >= self.threshold):
            # save the trigger time
            self.last_trig_time = time()
            # logging
            rospy.loginfo("Triggered! Correlation: %f" % corr_coef)
            rospy.loginfo(round_list(self.corr_pattern, 1))
            rospy.loginfo(round_list(list(self.mon_window), 1))
            rospy.loginfo(" --- ")
            # publishin of the event
            self.publish(dtype=MSG_TRIG)
        elif CONFIG_PRINT_HALF_CORR and (corr_coef >= self.threshold / 2):
            rospy.loginfo("Correlation: %f" % corr_coef)
        elif CONFIG_PRINT_CORR:
            rospy.loginfo("Correlation: %f" % corr_coef)

    def get_corr_coef(self):
        return self.last_corr_coef

    def get_light(self):
        return self.last_light

    def start_setup(self):
        """
        Parameters
        ----------
        corr_window_ms : int
        pattern : list
        threshold : int
        """
        def norm_minmax(a):
            norm_arr = []
            amin, amax = min(a), max(a)
            for i, val in enumerate(a):
                norm_arr.append((val - amin) / (amax - amin))
            return norm_arr

        def resize(in_list, new_size):
            # build rgb
            pat_rgb = []
            for i in in_list:
                pat_rgb.append([i, 0, 0])
            pixels = np.array([pat_rgb])
            # convert to image and resize
            new_image = Image.fromarray(pixels.astype('uint8'), 'RGB')
            image_resized = new_image.resize((new_size, 1))
            # retrieve resized list
            pixels = list(image_resized.getdata())
            lst = []
            for i in pixels:
                lst.append((i[0]))
            return lst

        def init_pattern(in_list, new_size):
            p_resized = resize(in_list, new_size)
            p_norm = norm_minmax(p_resized)
            return p_norm

        self.mon_window_sz_elements = self.init_window(self.mon_windows_sz_ms)
        self.corr_pattern = init_pattern(self.corr_pattern_raw, self.mon_window_sz_elements)
        rospy.loginfo("pattern ")
        rospy.loginfo(round_list(self.corr_pattern, 2))

    def poll_once(self):
        # read light
        self.last_light = self.read_light()
        if CONFIG_PRINT_LIGHT:
            rospy.loginfo("Light : " + hex(self.last_light))
        self.write_to_window(self.last_light)
        if CONFIG_PRINT_WINDOW and self.mon_window:
            rospy.loginfo("h'" + list2strf(list(self.mon_window), 5, in_hex=True))
        # publishing
        self.publish(dtype=MSG_SENSOR_VALUE, valA=self.last_light)
        # handling the sensor value
        if self.corr_pattern is not None:
            #  if there is enough contrast
            if calc_max_deviation(self.mon_window) > SHADOW_CONTRAST_PRST:
                self.last_corr_coef = np.corrcoef(self.corr_pattern, self.mon_window)[1, 0]
                self.trig_on_threshold(self.last_corr_coef)
            else:
                self.last_corr_coef = 0
