import rospy
from zakhar_i2c import ZkI2cDevice
from zakhar_pycore import zakhar__log as log
import threading
import collections
import numpy as np
from PIL import Image
from time import sleep, time

REG_VAL_LO = 0x02
REG_VAL_HI = 0x03

CONFIG_PRINT_HALF_CORR = True
CONFIG_PRINT_CORR = False
CONFIG_PRINT_WINDOW = False
CONFIG_PRINT_LIGHT = False
CONFIG_SHADOW_CONTRAST_PRST = 0.05
CONFIG_MINIMAL_TRIG_PERIOD_SEC = 1
# CONFIG_SHADOW_CONTRAST_PRST = 0.5

POLL_PERIOD = 0.01  # sec
WINDOWS_SIZE_SEC = 0.6  # sec
WINDOW_SIZE_ELEMENTS = int(WINDOWS_SIZE_SEC / POLL_PERIOD)


class SensorPhotoResistor(object):
    def __init__(self, sensor_platform, log_level):
        """
        Parameters
        ----------
        sensor_platform : ZkI2cDevice
        """
        self.l = log.get_logger("PhotoResistor")
        self.l.setLevel(log_level)
        self.sensor_platform = sensor_platform  # type:ZkI2cDevice
        self.light = 0
        self.polling = False
        self.poll_freq = 0
        self.mon_window = None
        self.corr_pattern = None
        self.corr_coef = 0
        self.threshold = None
        self.last_trig_time = -1

    def _read_light(self):
        try:
            lo = self.sensor_platform.read(REG_VAL_LO)
            hi = self.sensor_platform.read(REG_VAL_HI)
        except rospy.ServiceException:
            return 0xffff
        if lo is None or hi is None:
            return 0xffff
        val = (hi << 8) | lo
        if CONFIG_PRINT_LIGHT:
            self.l.info("Light : " + hex(val))
        return val

    def __upd_light(self):
        self.light = self._read_light()
        if self.mon_window is not None:
            if self.light != 0 and self.light != 0xffff:  # if the value is wrong - don't count it
                self.mon_window.append(self.light)
            if CONFIG_PRINT_WINDOW:
                self.l.info(
                    "h'" +
                    log.list2strf(list(self.mon_window), 5, in_hex=True))

    def _polling(self, freq):
        """
        Parameters
        ----------
        freq : int
        """
        self.poll_freq = freq
        self.l.info("Polling start")
        while 1:
            self.__upd_light()
            self.sensor_platform.publish(dtype="light", valA=self.light)
            self.__calc_corrcoef()
            sleep(1 / self.poll_freq)
            if not self.polling:
                break
        self.poll_freq = 0
        self.l.info("Polling end")

    def _init_window(self, size_ms):
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
        self.l.info("Windows inited, size is %d" % win_el_num)
        self.mon_window = collections.deque([0] * win_el_num,
                                            maxlen=win_el_num)
        return len(self.mon_window)

    def _deinit_window(self):
        self.mon_window = None

    def __patt_resize(self, in_list, new_size):
        #build rgb
        pat_rgb = []
        for i in in_list:
            pat_rgb.append([i, 0, 0])
        pixels = np.array([pat_rgb])
        #convert to image and resize
        new_image = Image.fromarray(pixels.astype('uint8'), 'RGB')
        image_resized = new_image.resize((new_size, 1))
        # retrieve resized list
        pixels = list(image_resized.getdata())
        lst = []
        for i in pixels:
            lst.append((i[0]))
        return lst

    def calc_max_deviation(self, l):
        lmax = float(max(l))
        lmin = float(min(l))
        delta = lmax - lmin
        if not delta:
            return 0
        return delta / lmax

    def round_list(self, l, num):
        new_l = []
        for i in l:
            new_l.append(round(i, num))
        return new_l

    def __calc_corrcoef(self):
        if self.corr_pattern is not None:
            if self.calc_max_deviation(
                    self.mon_window) > CONFIG_SHADOW_CONTRAST_PRST:
                self.corr_coef = np.corrcoef(self.corr_pattern,
                                             self.mon_window)[1, 0]
                if (self.corr_coef >= self.threshold) and (time() > self.last_trig_time + CONFIG_MINIMAL_TRIG_PERIOD_SEC):
                    self.last_trig_time = time()
                    self.l.info("Triggered! Correlation: %f" % self.corr_coef)
                    self.l.info(self.round_list(self.corr_pattern, 1))
                    self.l.info(self.round_list(list(self.mon_window), 1))
                    self.l.info(" --- ")
                    self.sensor_platform.publish(dtype="bird")
                elif self.corr_coef >= self.threshold / 2:
                    if CONFIG_PRINT_HALF_CORR:
                        self.l.info("Correlation: %f" % self.corr_coef)
                if CONFIG_PRINT_CORR:
                    self.l.info("Correlation: %f" % self.corr_coef)
            else:
                self.corr_coef = 0

    def norm_minmax(self, a):
        norm_arr = []
        amin, amax = min(a), max(a)
        for i, val in enumerate(a):
            norm_arr.append((val - amin) / (amax - amin))
        return norm_arr

    def start_corr_measurements(self, corr_window_ms, pattern, threshold):
        """
        Parameters
        ----------
        corr_window_ms : int
        pattern : list
        threshold : int
        """
        win_sz = self._init_window(corr_window_ms)
        p_resized = self.__patt_resize(pattern, win_sz)
        p_norm = self.norm_minmax(p_resized)
        self.corr_pattern = p_norm
        self.threshold = threshold
        self.l.info("pattern ")
        self.l.info(self.round_list(self.corr_pattern, 2))

    def get_corr_coef(self):
        return self.corr_coef

    def get_trig(self):
        """
        Returns
        -------
        bool
        """
        if self.corr_coef >= self.threshold:
            return True
        else:
            return False

    def get_light(self):
        return self.light

    def start_polling(self, freq=10):
        """
        Parameters
        ----------
        freq : int, optional
            Hz, by default 10
        """
        self.polling = True
        d = threading.Thread(name='[EYE]polling',
                             target=self._polling,
                             args=[freq])
        d.setDaemon(True)
        d.start()

    def stop_polling(self):
        self.polling = False
