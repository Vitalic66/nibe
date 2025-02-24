import logging
import serial
import time
import paho.mqtt.client as mqtt
from struct import unpack, pack

# Logger Setup
logging.basicConfig(level=logging.DEBUG, filename='nibe_debug.log', filemode='w',
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('NIBE')

# MQTT Setup
MQTT_BROKER = "192.168.178.92"
MQTT_PORT = 1883
MQTT_TOPIC_TRIGGER = "nibe/send_command"  # Home Assistant sendet 'send' an dieses Topic

mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(username="", password="")
mqtt_client.will_set("nibe/status", "offline", retain=True)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Serial Setup
SERIAL_PORT = "/dev/ttyUSB0"  # Passe dies an dein Gerät an
ser = serial.Serial(SERIAL_PORT, 19200, bytesize=serial.EIGHTBITS,
                    stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=3)

# Erwartete Trigger-Sequenz für den Start (in HEX)
TRIGGER_SEQUENCE = bytes.fromhex("A0 00 59 02 26 3E E3 06 03")

# Nachricht, die gesendet werden soll, wenn Trigger-Sequenz erkannt und MQTT-Trigger aktiv ist
SEND_MESSAGE = bytes.fromhex("D0 80 56 50 49 28 64 04 02 0A 9D 00")

# Status speichern, ob eine Nachricht gesendet werden soll
send_command_active = False

# MQTT Statusmeldung senden
def publish_mqtt(topic, message):
    mqtt_client.publish(topic, message)
    logger.info(f"Published {message} to {topic}")

# MQTT Availability publizieren
def publish_availability(status):
    mqtt_client.publish("nibe/status", status, retain=True)
    logger.info(f"Published availability status: {status}")

# MQTT Callback für empfangene Nachrichten
def on_message(client, userdata, msg):
    global send_command_active
    if msg.topic == MQTT_TOPIC_TRIGGER:
        payload = msg.payload.decode()
        if payload.lower() == "send":
            send_command_active = True
            logger.info("MQTT-Trigger empfangen: Nachricht wird gesendet, sobald die Trigger-Sequenz erkannt wird.")
        elif payload.lower() == "reset":
            send_command_active = False
            logger.info("MQTT-Trigger wurde zurückgesetzt.")

# MQTT Client Setup
mqtt_client.on_message = on_message
mqtt_client.subscribe(MQTT_TOPIC_TRIGGER)
mqtt_client.loop_start()

def wait_for_sequence():
    """ Wartet, bis die erwartete Sequenz empfangen wurde """
    logger.info(f"Warte auf Sequenz: {TRIGGER_SEQUENCE.hex(' ')}")
    buffer = bytearray()

    while True:
        byte = ser.read(1)  # Einzelnes Byte lesen
        if not byte:
            continue  # Falls Timeout oder keine Daten -> weiter warten

        buffer.append(byte[0])

        # Prüfen, ob das Ende der erwarteten Sequenz erreicht wurde
        if len(buffer) > len(TRIGGER_SEQUENCE):
            buffer.pop(0)  # Erstes Element entfernen, um Fenstergröße konstant zu halten

        if bytes(buffer) == TRIGGER_SEQUENCE:
            logger.info("Erwartete Sequenz empfangen!")
            return True

def send_serial_message():
    """ Sendet eine festgelegte Nachricht über die serielle Schnittstelle """
    try:
        # Sicherheitshalber Verbindung erneut öffnen
        if not ser.is_open:
            ser.open()

        ser.write(SEND_MESSAGE)
        ser.flush()
        logger.info(f"Nachricht gesendet: {SEND_MESSAGE.hex(' ')}")
        publish_mqtt("nibe/status", "Nachricht gesendet")
    except Exception as e:
        logger.error(f"Fehler beim Senden der Nachricht: {e}")

def run():
    logger.info("Starting the main loop...")
    publish_availability("online")  # Verfügbarkeit als online setzen

    try:
        while True:
            try:
                if wait_for_sequence():  # Erst warten, bis die Trigger-Sequenz empfangen wird
                    if send_command_active:  # Prüfen, ob ein MQTT-Trigger aktiv ist
                        send_serial_message()
                        send_command_active = False  # Einmalige Nachrichtensendung nach Empfang der Sequenz
                    else:
                        logger.info("Trigger-Sequenz empfangen, aber kein MQTT-Befehl erhalten.")
            except Exception as e:
                logger.warning(f"Fehler in der Verarbeitung: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Script beendet.")
    finally:
        publish_availability("offline")  # Verfügbarkeit als offline setzen
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        ser.close()

if __name__ == "__main__":
    logger.info("Starting Nibe heat pump MQTT bridge")
    run()
