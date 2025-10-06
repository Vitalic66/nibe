#!/usr/bin/python3
import sys
import paho.mqtt.client as mqtt
import ssl

# set the variables
# Path to the Sensor systempath
# 28-01142f7ba71a has to be changed to you sensor path!
#6cd - intern
#4bd - extern
sensor = '/sys/bus/w1/devices/28-6cd3211864ff/w1_slave'
sensor2 = '/sys/bus/w1/devices/28-4bd4211864ff/w1_slave'
broker='192.168.178.92'
port=1883
#publish_topic="house/pi-ds18b20"
publish_topic="nibe/"
clientid='python-mqtt-ds18b20'
username=''
password=''
insecure=True
qos=1
retain_message=True

# do the stuff

#def connect_mqtt():
#    def on_connect(client, userdata, flags, rc):
#        if rc == 0:
#            print("Connected to MQTT Broker!")
#        else:
#            print("Failed to connect, return code %d\n", rc)
#
#    client = mqtt_client.Client(client_id)
#    # client.username_pw_set(username, password)
#    client.on_connect = on_connect
#    client.connect(broker, port)
#    return client

def readTempSensor(sensorName) :
    f = open(sensorName, 'r')
    lines = f.readlines()
    f.close()
    return lines

def readTempLines(sensorName) :
    lines = readTempSensor(sensorName)
    #time.sleep(60)
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = readTempSensor(sensorName)
    temperaturStr = lines[1].find('t=')
    if temperaturStr != -1 :
        tempData = lines[1][temperaturStr+2:]
        if sensorName == '/sys/bus/w1/devices/28-6cd3211864ff/w1_slave' :
            tempCelsius = float(tempData) / 1000.0 + 1.9
        if sensorName == '/sys/bus/w1/devices/28-4bd4211864ff/w1_slave' :
            tempCelsius = float(tempData) / 1000 - 1.0
#        tempKelvin = 273 + float(tempData) / 1000
#        tempFahrenheit = float(tempData) / 1000 * 9.0 / 5.0 + 32.0
#        return [tempCelsius, tempKelvin, tempFahrenheit]
        return [tempCelsius]


#MQTT Connection
client=mqtt.Client(clientid)
client.username_pw_set(username, password)
#client.tls_set(cert_reqs=ssl.CERT_NONE) #no client certificate needed
#client.tls_insecure_set(insecure)
client.connect(broker, port)
client.loop_start()
#time.sleep(60)
client.publish("{}brauchwassertemp".format(publish_topic),"{:.1f}".format(readTempLines(sensor)[0]),qos,retain_message)
client.publish("{}ofentank".format(publish_topic),"{:.1f}".format(readTempLines(sensor2)[0]),qos,retain_message)

client.disconnect()
client.loop_stop()
