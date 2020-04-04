#!/usr/bin/python

# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
import VL53L0X

class vl53l0x():

    def __init__(self):

        # Create a VL53L0X object
        self._ser = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
        self._ser.open()
        self._ser.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

        self._distance = 0
        self.distance_min   = 3
        self.distance_max   = 200

    def get_data(self):

        time0 = time.time()
        while True:
            if time.time() > time0 + 1: break
            distance = self._ser.get_distance()

        self._distance = distance
        return(distance)

    @property
    def distance(self):
        return(self._distance)

    def print_data_thread(self):
        while True:
            print(self.get_data())
            time.sleep(0.5)

    def close(self):
        if self._ != None:
            self._ser.stop_ranging()
            self._ser.close()


if __name__ == '__main__':
    tof = vl53l0x()
    # print vl53l0x.get_data()
    tof.print_data_thread()
