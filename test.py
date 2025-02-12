#!/usr/bin/env python3
# vim: set encoding=utf-8 tabstop=4 softtabstop=4 shiftwidth=4 expandtab
import logging
import serial
from struct import unpack, pack

logger = logging.getLogger('NIBE')

class NIBE():
    def __init__(self, smarthome, serialport):
        self._sh = smarthome
        self._nibe_regs = {}
        
        try:
            self._serial = serial.Serial(serialport, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_MARK, timeout=3)
        except serial.SerialException as e:
            logger.error(f"Fehler beim Öffnen des seriellen Ports: {e}")
            self._serial = None
    
    def run(self):
        if self._serial is None:
            logger.error("Serielle Verbindung nicht initialisiert.")
            return
        
        self.alive = True
        try:
            while self.alive:
                expected_sequence = b"\x55\x00\x59\x02\xC2\x2A\xE6\x06\x03\x00\xF9\x06\x03"
                received_sequence = self._serial.read(len(expected_sequence))
                if len(received_sequence) < len(expected_sequence):
                    logger.warning("Unvollständige Nachricht empfangen")
                    continue
                
                if received_sequence != expected_sequence:
                    continue
                
                try:
                    self._serial.write(b"\x00\xF1\x06\x8C\x00\x59\x02\x04\x80\x53\x06\x03\x00\xF1\x06\xA0\x00\x59\x02\x26\x3E\xE3\x06\x03\x00\xF9\x06\xD0\x00\x59\x07\x25\x02\x09\x16\x10\x00\x01\xA7")
                    self.alive = False  # Setze alive auf False nach erfolgreichem Schreiben
                except serial.SerialException as e:
                    logger.error(f"Fehler beim Schreiben auf den seriellen Port: {e}")
                    continue
                
        except serial.SerialException as e:
            logger.error(f"Serielle Verbindung fehlgeschlagen: {e}")
        except Exception as e:
            logger.warning(f"Allgemeiner Fehler in der run-Methode: {e}")
    
    def stop(self):
        self.alive = False
        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException as e:
                logger.error(f"Fehler beim Schließen des seriellen Ports: {e}")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    nibe = NIBE(None, "/dev/ttyUSB0")
    try:
        nibe.run()
    except KeyboardInterrupt:
        nibe.stop()
