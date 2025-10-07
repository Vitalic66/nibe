#!/usr/bin/python3
import sys
import time
import paho.mqtt.client as mqtt
import ssl

# ---- ADS1115 / Drucksensor ----
try:
    import Adafruit_ADS1x15
except ImportError:
    print("Fehler: 'Adafruit_ADS1x15' ist nicht installiert. -> pip3 install Adafruit-ADS1x15")
    sys.exit(1)

# ---------------- MQTT / DS18B20 CONFIG ----------------
# 28-01142f7ba71a has to be changed to your sensor path!
sensor  = '/sys/bus/w1/devices/28-6cd3211864ff/w1_slave'  # intern
sensor2 = '/sys/bus/w1/devices/28-4bd4211864ff/w1_slave'  # extern  (DEIN Pfad oben hat 4bd..., prüfe!)
broker='192.168.178.92'
port=1883
publish_prefix="nibe/"
clientid='python-mqtt-ds18b20'
username=''
password=''
insecure=True
qos=1
retain_message=True

# ---------------- Drucksensor / ADS1115 CONFIG ----------------
ADS_ADDR   = 0x48
I2C_BUS    = 1
ADC_CH     = 0
GAIN       = 2/3  # Skala bis 6.144 V (Achtung: Eingänge dürfen nie über VDD)
VFULLSCALE = 6.144  # zu GAIN passend

# Sensor-Kennlinie: 0–80 psi, 0.5–4.5 V ratiometrisch
PSI_MAX = 80.0
V_MIN   = 0.5
V_MAX   = 4.5
SLOPE   = PSI_MAX / (V_MAX - V_MIN)  # = 20.0
CAL_OFFSET_PSI = 1.9  # kleinen Offset zur Angleichung an Manometer hier setzen (z.B. +1.9)

# Mittelung Samples für Druck
PRESSURE_SAMPLES = 10

# ---------------- Helpers ----------------
def readTempSensor(sensorName):
    with open(sensorName, 'r') as f:
        lines = f.readlines()
    return lines

def readTempLines(sensorName):
    lines = readTempSensor(sensorName)
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = readTempSensor(sensorName)
    temperaturStr = lines[1].find('t=')
    if temperaturStr != -1:
        tempData = lines[1][temperaturStr+2:]
        # individuelle Korrekturen
        if sensorName == '/sys/bus/w1/devices/28-6cd3211864ff/w1_slave':
            tempCelsius = float(tempData) / 1000.0 + 1.9
        elif sensorName == '/sys/bus/w1/devices/28-4bd4211864ff/w1_slave':
            tempCelsius = float(tempData) / 1000.0 - 1.0
        else:
            tempCelsius = float(tempData) / 1000.0
        return [tempCelsius]
    return [None]

def read_pressure_ads1115():
    """Liest den ADS1115 ein, mittelt und gibt (psi, bar, volts_avg, raw_last) zurück."""
    adc = Adafruit_ADS1x15.ADS1115(address=ADS_ADDR, busnum=I2C_BUS)
    acc_v = 0.0
    raw_last = 0
    for _ in range(PRESSURE_SAMPLES):
        raw = adc.read_adc(ADC_CH, gain=GAIN)  # single-ended 0..32767
        volts = (raw / 32767.0) * VFULLSCALE
        acc_v += volts
        raw_last = raw
        time.sleep(0.005)
    volts_avg = acc_v / PRESSURE_SAMPLES

    # in psi abbilden (clamping)
    if volts_avg <= V_MIN:
        psi = 0.0
    elif volts_avg >= V_MAX:
        psi = PSI_MAX
    else:
        psi = (volts_avg - V_MIN) * SLOPE

    psi += CAL_OFFSET_PSI
    bar = psi * 0.0689475729
    return psi, bar, volts_avg, raw_last

# ---------------- MQTT publish ----------------
def publish_once():
    # MQTT Connection
    client = mqtt.Client(clientid)
    if username or password:
        client.username_pw_set(username, password)
    # client.tls_set(cert_reqs=ssl.CERT_NONE)
    # client.tls_insecure_set(insecure)
    client.connect(broker, port)
    client.loop_start()

    # Temperaturen
    t1 = readTempLines(sensor)[0]
    t2 = readTempLines(sensor2)[0]

    # Druck
    psi, bar, volts, raw = read_pressure_ads1115()

    # Publish
    if t1 is not None:
        client.publish(f"{publish_prefix}brauchwassertemp", f"{t1:.1f}", qos, retain=False)
    if t2 is not None:
        client.publish(f"{publish_prefix}ofentank", f"{t2:.1f}", qos, retain=False)

    client.publish(f"{publish_prefix}druck_bar", f"{bar:.2f}", qos, retain=False)
    client.publish(f"{publish_prefix}druck_psi", f"{psi:.1f}", qos, retain=False)
    # Debug optional:
    client.publish(f"{publish_prefix}druck_volt", f"{volts:.3f}", qos, retain=False)
    client.publish(f"{publish_prefix}druck_raw", str(raw), qos, retain=False)

    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    try:
        publish_once()
    except Exception as e:
        print(f"Fehler: {e}")
        sys.exit(1)
