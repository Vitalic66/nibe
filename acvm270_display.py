import logging
import serial
import time
import paho.mqtt.client as mqtt
from struct import unpack, pack

# Setup logger
#logging.basicConfig(level=logging.WARNING)
logging.basicConfig(level=logging.DEBUG, filename='nibe_display_debug.log', filemode='w', format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('NIBE')

# MQTT setup
mqtt_client = mqtt.Client()
mqtt_client.reconnect_delay_set(min_delay=1, max_delay=60)
mqtt_client.username_pw_set(username="", password="")
mqtt_client.connect("192.168.178.92", 1883, 60)

def publish_mqtt(topic, message):
    mqtt_client.publish(topic, message)
    logger.info(f"Published {message} to {topic}")

# Serial connection to Nibe heat pump (adjust COM port for Windows)
serial_port = "/dev/ttyUSB1"  # Adjust to your correct COM port
ser = serial.Serial(serial_port, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=3)

logger.debug("Serial port opened successfully")


def publish_ascii_message_with_subtopic(identifier: int, payload_bytes: bytes):
    """
    Wandelt die übergebenen Payload-Bytes in einen ASCII-String um und publiziert diesen
    an ein bestimmtes MQTT-Subtopic, basierend auf dem übergebenen Identifier.
    """
    #try:
        # Bestimme das Subtopic basierend auf dem Identifier
    if identifier == 0x51:
        topic = "nibe/display/line2"
    elif identifier == 0x52:
        topic = "nibe/display/line3"
    elif identifier == 0x53:
        topic = "nibe/display/line4"
    elif identifier == 0x50:
        topic = "nibe/display/line1"
    else:
        topic = "nibe/display/unknown"
        
        # Konvertiere die Bytes in einen ASCII-String
    ascii_payload = payload_bytes.decode('ascii', errors='replace')
    publish_mqtt(topic, ascii_payload)
    logger.debug(f"Published ASCII message '{ascii_payload}' to topic '{topic}'")
    #except Exception as e:
        #logger.error(f"Fehler beim Verarbeiten der ASCII-Nachricht: {e}")

def run():
    logger.info("Starting the main loop...")

    mqtt_client.loop_start()
    try:
        while True:
            try:
                if ser.read(1)[0] != 0x03:
                    logger.debug("No start byte found")
                    continue
                logger.debug("Start byte found, reading data...")
                ret = ser.read(2)
                if ret[0] != 0x00 and ret[1] != 0xf9 :
                    logger.debug("Invalid start frame")
                    continue
                ack = ser.read(1)
                if not ack or ack[0] != 0x06:
                    continue
                logger.debug(f"ack found : {ack}")
                frm = ser.read(4)
                if frm[0] == 0x03:
                    continue
                if frm[2] != 0x59:
                    continue        #ignore D4 message
                l = int(frm[3])
                dl = frm[0] # 0x50 = Zeile 1 (Symbole), 0x51 = Zeile 2, 0x52 = Zeile 3, 0x53 = Zeile 4
                logger.debug(f"displayline: {dl}")
                frm += ser.read(l + 1)
                crc = 0
                for i in frm[:-1]:          
                    crc ^= i
                if crc != frm[-1]:
                    logger.debug("Frame CRC error")
                    continue
                msg = frm[4:-1] #letztes byte = müll
                logger.debug(f"msg: {msg}")
                publish_ascii_message_with_subtopic(dl, msg)
                ser.reset_input_buffer()
                continue
            except Exception as e:
                logger.debug(f"Error in Nibe data processing: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Script interrupted, shutting down...")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

if __name__ == "__main__":
    logger.info("Starting Nibe heat pump display MQTT bridge")
    run()
