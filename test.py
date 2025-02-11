#!/usr/bin/env python3
# vim: set encoding=utf-8 tabstop=4 softtabstop=4 shiftwidth=4 expandtab
#########################################################################
#  Copyright 2013 KNX-User-Forum e.V.           https://knx-user-forum.de/
#########################################################################
#  NIBE plugin for SmartHome.py.         http://mknx.github.io/smarthome/
#
#  This plugin is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This plugin is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this plugin. If not, see <http://www.gnu.org/licenses/>.
#########################################################################

import logging
import serial
import re
#import termios
#from struct import *
from struct import unpack, pack

#logger = logging.getLogger('NIBE')

class NIBE():
    def __init__(self, smarthome, serialport):
        #self._sh = smarthome
        #self._nibe_regs = {}
        
        
        #self._serial = serial.Serial(serialport, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, timeout=2)
        #iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self._serial)
        #CMSPAR = 0x40000000
        #cflag |= termios.PARENB | CMSPAR | termios.PARODD # to select MARK parity
        #termios.tcsetattr(self._serial, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
        
        #serial_port = "/dev/ttyUSB0"  # Adjust to your correct COM port
        #ser = serial.Serial(serial_port, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_MARK, timeout=3)
        
        self._serial = serial.Serial(/dev/ttyUSB0, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_MARK, timeout=3)
          
    def run(self):
        self.alive = True
        try:
            while self.alive:
                #suche nach Endsequenz 55 00 59 02 C2 2A E6 06 03 00 F9 06 03
                if bytes(self._serial.read(1))[0] != 0x55:
                    continue
                if bytes(self._serial.read(1))[0] != 0x00:
                    continue
                #if bytes(self._serial.read(1))[0] != 0x14:
                #    continue
                #self._serial.write(b"\x06")
                #self._serial.drainOutput()
                #self._serial.flushInput()
                if bytes(self._serial.read(1))[0] != 0x59:
                    continue
                if bytes(self._serial.read(1))[0] != 0x02:
                    continue
                if bytes(self._serial.read(1))[0] != 0xC2:
                    continue
                if bytes(self._serial.read(1))[0] != 0x2A:
                    continue
                if bytes(self._serial.read(1))[0] != 0xE6:
                    continue
                if bytes(self._serial.read(1))[0] != 0x06:
                    continue
                if bytes(self._serial.read(1))[0] != 0x03:
                    continue
                if bytes(self._serial.read(1))[0] != 0x00:
                    continue
                if bytes(self._serial.read(1))[0] != 0xF9:
                    continue
                if bytes(self._serial.read(1))[0] != 0x06:
                    continue
                if bytes(self._serial.read(1))[0] != 0x03:
                    continue
                self._serial.write(b"\x00\xF1\x06\xF6\x8C\x00\x59\x02\x04\x80\x53\x06\x03\x00\xF1\x06\xA0\x00\x59\x02\x26\x3E\xE3\x06\x03\x00\xF9\x06\xD0\x00\x59\x07\x25\x02\x09\x16\x10\x00\x01\xA7")    
                self._serial.drainOutput()
                self._serial.flushInput()
                self._serial.close()
                        
        #except Exception as e:
        #    logger.warning("nibe: {0}".format(e))

    def stop(self):
        self.alive = False
        self._serial.close()
        
if __name__ == "__main__":
    #logger.info("Starting Nibe heat pump MQTT bridge")
    run()
