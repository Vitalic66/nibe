import logging
import serial
import time
import paho.mqtt.client as mqtt
from struct import unpack, pack

# Setup logger
#logging.basicConfig(level=logging.WARNING)
logging.basicConfig(level=logging.WARNING, filename='nibe_debug.log', filemode='w', format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('NIBE')

# MQTT Setup
MQTT_BROKER = "192.168.178.92"
MQTT_PORT = 1883
MQTT_TOPIC_TRIGGER = "nibe/send_command"

mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(username="", password="")
mqtt_client.will_set("nibe/status", "offline", retain=True)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

def publish_mqtt(topic, message):
    mqtt_client.publish(topic, message)
    logger.info(f"Published {message} to {topic}")

# Publish "online" status to availability topic
def publish_availability(status):
    mqtt_client.publish("nibe/status", status, retain=True)
    logger.info(f"Published availability status: {status}")

# Serial connection to Nibe heat pump (adjust COM port for Windows)
serial_port = "/dev/ttyUSB0"  # Adjust to your correct COM port
ser = serial.Serial(serial_port, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=3)

logger.debug("Serial port opened successfully")

# Erwartete Sequenz zur Aktivierung des Sendebefehls
TRIGGER_SEQUENCE = bytes.fromhex("A0 00 59 02 26 3E E3 06 03")

# Nachricht, die gesendet wird, wenn Trigger-Sequenz erkannt wurde
SEND_MESSAGE = bytes.fromhex("D0 80 56 50 49 28 64 04 02 0A 9D 00")

# Variable zur Steuerung des MQTT-gestützten Sendebefehls
send_command_active = False

# Callback für MQTT-Nachrichten (Empfang von `nibe/send_command`)
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
import logging
import serial
import time
import paho.mqtt.client as mqtt
from struct import unpack, pack

# Setup logger
#logging.basicConfig(level=logging.WARNING)
logging.basicConfig(level=logging.WARNING, filename='nibe_debug.log', filemode='w', format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('NIBE')

# MQTT Setup
MQTT_BROKER = "192.168.178.92"
MQTT_PORT = 1883
MQTT_TOPIC_TRIGGER = "nibe/send_command"

mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(username="", password="")
mqtt_client.will_set("nibe/status", "offline", retain=True)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

def publish_mqtt(topic, message):
    mqtt_client.publish(topic, message)
    logger.info(f"Published {message} to {topic}")

# Publish "online" status to availability topic
def publish_availability(status):
    mqtt_client.publish("nibe/status", status, retain=True)
    logger.info(f"Published availability status: {status}")

# Serial connection to Nibe heat pump (adjust COM port for Windows)
serial_port = "/dev/ttyUSB0"  # Adjust to your correct COM port
ser = serial.Serial(serial_port, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=3)

logger.debug("Serial port opened successfully")

# Erwartete Sequenz zur Aktivierung des Sendebefehls
TRIGGER_SEQUENCE = bytes.fromhex("A0 00 59 02 26 3E E3 06 03")

# Nachricht, die gesendet wird, wenn Trigger-Sequenz erkannt wurde
SEND_MESSAGE = bytes.fromhex("D0 80 56 50 49 28 64 04 02 0A 9D 00")

# Variable zur Steuerung des MQTT-gestützten Sendebefehls
send_command_active = False

# Callback für MQTT-Nachrichten (Empfang von `nibe/send_command`)
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

# Register mapping, expanded to match original script
nibe_registers = {
    0: "nibe/cpu_id",
    1: "nibe/outdoor_temp_c",  
    4: "nibe/heating_curve",
    5: "nibe/flow_setpoint_c",  
    6: "nibe/flow_actual_c",  
    7: "nibe/return_temp_c",
    8: "nibe/degree_minutes",
    12: "nibe/domestic_hot_water_top_temp",
    25: "nibe/compressor_starts",
    31: "nibe/heating_status",
    32: "nibe/additional_heating_allowed",
    33: "nibe/max_df_compressor",
    34: "nibe/verd_freq_reg_p",
    35: "nibe/min_start_time_freq_min",
    36: "nibe/min_time_const_freq_min",
    38: "nibe/comp_freq_grad_min",
    44: "nibe/pump_speed_percent",
    45: "nibe/bw_reg_p",
    46: "nibe/bw_reg_q",
    48: "nibe/bw_reg_value_xp_percent",
    100: "nibe/date_year",
    101: "nibe/date_month",
    102: "nibe/date_day",
    103: "nibe/time_hour",
    104: "nibe/time_minute",
    105: "nibe/time_second",
    24: "nibe/run_time_compressor_h",
    9: "nibe/comp_freq_desired_hz",
    10: "nibe/comp_freq_actual_hz",
    19: "nibe/high_pressure_bar",
    20: "nibe/low_pressure_bar",
    22: "nibe/ams_phase_is_a",
    40: "nibe/hysteresis",
    47: "nibe/bw_reg_xp",
    43: "nibe/stop_temp_heating_c",
    49: "nibe/domestic_hot_water_start_temp",
    50: "nibe/domestic_hot_water_stop_temp",
    28: "nibe/operation_mode_reg_28",
    29: "nibe/operation_mode_reg_29",
    30: "nibe/operation_mode_reg_30",
    23: "nibe/inverter_temp_tho_ip",
    11: "nibe/condenser_off_max",
    13: "nibe/domestic_hot_water_bottom",
    14: "nibe/tho_r1_evap_temp",
    15: "nibe/tho_r2_evap_temp",
    16: "nibe/suction_gas_temp_tho_s",
    17: "nibe/hot_gas_temp_tho_d",
    18: "nibe/liquid_temp_ams",
    21: "nibe/resp_at_ams_tho_a",
    #############################
    128: "nibe/8b_temp1",
    129: "nibe/8b_temp2",
    131: "nibe/8b_temp3",
    135: "nibe/8b_temp4",
}
  

# Define unique IDs and MQTT discovery configurations for each sensor
mqtt_discovery_sensors = {
    "nibe/pump_speed_percent": {
        "name": "Pumpengeschwindigkeit",
        "unit_of_measurement": "%",
        "state_topic": "nibe/pump_speed_percent",
        "unique_id": "nibe_pump_speed_percent"
    },
#    "nibe/cpu_id": {
#        "name": "CPU ID",
#        "state_topic": "nibe/cpu_id",
#        "unique_id": "nibe_cpu_id"
#    },
    "nibe/inverter_temp_tho_ip": {
        "name": "Invertertemp. Tho-IP",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/inverter_temp_tho_ip",
        "unique_id": "nibe_inverter_temp_tho_ip"
    },
    "nibe/operation_mode": {
        "name": "Nibe Operation Mode",
        "state_topic": "nibe/operation_mode",
    "unique_id": "nibe_operation_mode"
    },
    "nibe/condenser_off_max": {
        "name": "Kondensator aus (MAX)",
        "state_topic": "nibe/condenser_off_max",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_condenser_off_max"
    },
    "nibe/domestic_hot_water_bottom": {
        "name": "Brauchwasser unten",
        "state_topic": "nibe/domestic_hot_water_bottom",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_domestic_hot_water_bottom"
    },
    "nibe/tho_r1_evap_temp": {
        "name": "Verd. Temp. Tho-R1",
        "state_topic": "nibe/tho_r1_evap_temp",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_tho_r1_evap_temp"
    },
    "nibe/tho_r2_evap_temp": {
        "name": "Verd. Temp. Tho-R2",
        "state_topic": "nibe/tho_r2_evap_temp",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_tho_r2_evap_temp"
    },
    "nibe/suction_gas_temp_tho_s": {
        "name": "Sauggastemp. Tho-S",
        "state_topic": "nibe/suction_gas_temp_tho_s",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_suction_gas_temp_tho_s"
    },
    "nibe/hot_gas_temp_tho_d": {
        "name": "Heissgastemp. Tho-D",
        "state_topic": "nibe/hot_gas_temp_tho_d",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_hot_gas_temp_tho_d"
    },
    "nibe/liquid_temp_ams": {
        "name": "Flüssigkeitstemperatur AMS",
        "state_topic": "nibe/liquid_temp_ams",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_liquid_temp_ams"
    },
    "nibe/resp_at_ams_tho_a": {
        "name": "Atemp. am AMS Tho-A",
        "state_topic": "nibe/resp_at_ams_tho_a",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_resp_at_ams_tho_a"
    },     
    "nibe/heating_curve": {
        "name": "Heizkurvenverschiebung",
        "state_topic": "nibe/heating_curve",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_heating_curve"
    },
    "nibe/return_temp_c": {
        "name": "Rücklauf",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/return_temp_c",
        "unique_id": "nibe_return_temp_c"
    },
    "nibe/degree_minutes": {
        "name": "Gradminuten",
        "state_topic": "nibe/degree_minutes",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_degree_minutes"
    },
    "nibe/domestic_hot_water_top_temp": {
        "name": "Brauchwasser oben",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/domestic_hot_water_top_temp",
        "unique_id": "nibe_domestic_hot_water_top_temp"
    },
    "nibe/max_df_compressor": {
        "name": "Max dF Verdichter",
        "state_topic": "nibe/max_df_compressor",
        "unit_of_measurement": "Hz",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_max_df_compressor"
    },
    "nibe/verd_freq_reg_p": {
        "name": "Verd. Freq. reg P",
        "state_topic": "nibe/verd_freq_reg_p",
        "unit_of_measurement": "Hz",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_verd_freq_reg_p"
    },
    "nibe/min_start_time_freq_min": {
        "name": "Min Startzeit Freq",
        "unit_of_measurement": "Hz",
        "state_topic": "nibe/min_start_time_freq_min",
        "unique_id": "nibe_min_start_time_freq_min"
    },
    "nibe/min_time_const_freq_min": {
        "name": "Minzeit konst. Freq",
        "state_topic": "nibe/min_time_const_freq_min",
        "unit_of_measurement": "Hz",
        "unique_id": "nibe_min_time_const_freq_min"
    },
    "nibe/comp_freq_grad_min": {
        "name": "Verd. Freq. GradMin",
        "state_topic": "nibe/comp_freq_grad_min",
        "unit_of_measurement": "Hz",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_comp_freq_grad_min"
    },
    "nibe/bw_reg_p": {
        "name": "Bw reg P",
        "state_topic": "nibe/bw_reg_p",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_bw_reg_p"
    },
    "nibe/bw_reg_q": {
        "name": "Bw reg Q",
        "state_topic": "nibe/bw_reg_q",
        "value_template": "{{ value | float }}",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_bw_reg_q"
    },
    "nibe/bw_reg_value_xp_percent": {
        "name": "Bw reg Wert xP",
        "unit_of_measurement": "%",
        "state_topic": "nibe/bw_reg_value_xp_percent",
        "unique_id": "nibe_bw_reg_value_xp_percent"
    },

#enable if you want mqtt to publish time and date
#    "nibe/date_year": {
#        "name": "Date Year",
#        "state_topic": "nibe/date_year",
#        "unique_id": "nibe_date_year"
#    },
#    "nibe/date_month": {
#        "name": "Date Month",
#        "state_topic": "nibe/date_month",
#        "unique_id": "nibe_date_month"
#    },
#    "nibe/date_day": {
#        "name": "Date Day",
#        "state_topic": "nibe/date_day",
#        "unique_id": "nibe_date_day"
#    },
#    "nibe/time_hour": {
#        "name": "Time Hour",
#        "state_topic": "nibe/time_hour",
#        "unique_id": "nibe_time_hour"
#    },
#    "nibe/time_minute": {
#        "name": "Time Minute",
#        "state_topic": "nibe/time_minute",
#        "unique_id": "nibe_time_minute"
#    },
#    "nibe/time_second": {
#        "name": "Time Second",
#        "state_topic": "nibe/time_second",
#        "unique_id": "nibe_time_second"
#    },
    "nibe/run_time_compressor_h": {
        "name": "Verdichterlaufzeit",
        "state_topic": "nibe/run_time_compressor_h",
        "device_class": "duration",
        "unit_of_measurement": "h",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_run_time_compressor_h"
    },
    "nibe/comp_freq_desired_hz": {
        "name": "Verd. Freq. Soll",
        "unit_of_measurement": "Hz",
        "state_topic": "nibe/comp_freq_desired_hz",
        "unique_id": "nibe_comp_freq_desired_hz"
    },
    "nibe/comp_freq_actual_hz": {
        "name": "Verd. Freq. Ist",
        "unit_of_measurement": "Hz",
        "state_topic": "nibe/comp_freq_actual_hz",
        "unique_id": "nibe_comp_freq_actual_hz"
    },
    "nibe/high_pressure_bar": {
        "name": "Hochdruck",
        "unit_of_measurement": "bar",
        "state_topic": "nibe/high_pressure_bar",
        "unique_id": "nibe_high_pressure_bar"
    },
    "nibe/low_pressure_bar": {
        "name": "Niederdruck",
        "unit_of_measurement": "bar",
        "state_topic": "nibe/low_pressure_bar",
        "unique_id": "nibe_low_pressure_bar"
    },
    "nibe/ams_phase_is_a": {
        "name": "AMS Phase Ist",
        "unit_of_measurement": "A",
        "state_topic": "nibe/ams_phase_is_a",
        "unique_id": "nibe_ams_phase_is_a"
    },
    "nibe/hysteresis": {
        "name": "Hystere",
        "state_topic": "nibe/hysteresis",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_hysteresis"
    },
    "nibe/bw_reg_xp": {
        "name": "Bw reg xP",
        "state_topic": "nibe/bw_reg_xp",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_bw_reg_xp"
    },
    "nibe/stop_temp_heating_c": {
        "name": "Stopptemperatur Heizen",
        "unit_of_measurement": "°C",
        "state_topic": "nibe/stop_temp_heating_c",
        "unique_id": "nibe_stop_temp_heating_c"
    },
    "nibe/domestic_hot_water_start_temp": {
        "name": "Brauchwasser Starttemperatur",
        "unit_of_measurement": "°C",
        "state_topic": "nibe/domestic_hot_water_start_temp",
        "unique_id": "nibe_domestic_hot_water_start_temp"
    },
    "nibe/domestic_hot_water_stop_temp": {
        "name": "Brauchwasser Stoptemperatur",
        "unit_of_measurement": "°C",
        "state_topic": "nibe/domestic_hot_water_stop_temp",
        "unique_id": "nibe_domestic_hot_water_stop_temp"
    },
    "nibe/outdoor_temp_c": {
        "name": "Außentemperatur",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/outdoor_temp_c",
        "unique_id": "nibe_outdoor_temp_c"
    },
    "nibe/flow_setpoint_c": {
        "name": "Vorlauf Soll",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/flow_setpoint_c",
        "unique_id": "nibe_flow_setpoint_c"
    },
    "nibe/flow_actual_c": {
        "name": "Vorlauf Ist",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/flow_actual_c",
        "unique_id": "nibe_flow_actual_c"
    },
    "nibe/compressor_starts": {
        "name": "Verdichterstarts",
        "state_topic": "nibe/compressor_starts",
        "unit_of_measurement": "x",
        "state_class": "total_increasing",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_compressor_starts"
    },
    "nibe/heating_status": {
        "name": "Status Heizung",
        "state_topic": "nibe/heating_status",
        "unique_id": "nibe_heating_status"
    },
    ######################################################
    "nibe/8b_temp1": {
        "name": "8b Temp1",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp1",
        "unique_id": "nibe_8b_temp1"
    },
    "nibe/8b_temp2": {
        "name": "8b Temp2",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp2",
        "unique_id": "nibe_8b_temp2"
    },
    "nibe/8b_temp3": {
        "name": "8b Temp3",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp3",
        "unique_id": "nibe_8b_temp3"
    },
    "nibe/8b_temp4": {
        "name": "8b Temp4",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp4",
        "unique_id": "nibe_8b_temp4"
    },
######################################################
    "nibe/additional_heating": {
        "name": "Zusatzheizung erlaubt",
        "state_topic": "nibe/additional_heating_allowed",
        "unique_id": "nibe_additional_heating_allowed",
        "value_template": "{{ value | int }}",  # Ensure Home Assistant interprets 0 and 1 as integers
        "availability_topic": "nibe/status",
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": {
            "identifiers": ["nibe_heat_pump"],
            "name": "Nibe Wärmepumpe",
            "manufacturer": "Nibe",
            "model": "ACVM 270",
            "sw_version": "1.0"
    }
},

}


# Publish MQTT discovery payload for each sensor
def publish_discovery_payloads():
    for sensor, config in mqtt_discovery_sensors.items():
        discovery_topic = f"homeassistant/sensor/{config['unique_id']}/config"
        
        # Build the base payload
        payload = {
            "name": config["name"],
            "state_topic": config["state_topic"],
            "unique_id": config["unique_id"],
            "availability_topic": "nibe/status",
            "payload_available": "online",
            "payload_not_available": "offline",
            "device": {
                "identifiers": ["nibe_heat_pump"],
                "name": "Nibe ACVM 270 Split",
                "manufacturer": "Nibe",
                "model": "ACVM 270",
                "sw_version": "1.0"
            }
        }

        # Add optional fields only if they exist
        if config.get("unit_of_measurement"):
            payload["unit_of_measurement"] = config["unit_of_measurement"]
        if config.get("device_class"):
            payload["device_class"] = config["device_class"]
        if config.get("value_template"):
            payload["value_template"] = config["value_template"]

        # Publish the discovery payload as JSON
        mqtt_client.publish(discovery_topic, str(payload).replace("'", '"'), qos=0, retain=True)
        logger.info(f"Published MQTT discovery payload for {config['name']}")

# Initialize global variables to store register 28, 29, and 30 values
reg28_value = None
reg29_value = None
reg30_value = None

def _decode(reg, raw):
    global reg28_value, reg29_value, reg30_value  # Ensure these variables are accessible
    
    if len(raw) == 2:
        value = unpack('>H', raw)[0]
    else:
        value = unpack('B', raw)[0]

    # Handle registers 28, 29, and 30
    if reg == 28:
        reg28_value = value
        logger.debug(f"Register 28 (operation mode) value: {reg28_value}")
        return None  # Avoid falling through to unhandled case
    elif reg == 29:
        reg29_value = value
        logger.debug(f"Register 29 (operation mode) value: {reg29_value}")
        return None  # Avoid falling through to unhandled case
    elif reg == 30:
        reg30_value = value
        logger.debug(f"Register 30 (operation mode) value: {reg30_value}")

        # Now that we have all three registers, interpret the state
        if reg28_value == 0x0000 and reg29_value == 0x8222 and reg30_value == 0x0032:
            publish_mqtt("nibe/operation_mode", "Standby") #pwer on, heatpump off
        elif reg28_value == 0x4409 and reg29_value == 0xA22A and reg30_value == 0x01FE:
            publish_mqtt("nibe/operation_mode", "Brauchwasserbereitung, AMS an") #domestic water
        elif reg28_value == 0x0008 and reg29_value == 0xC22A and reg30_value == 0x000A:
            publish_mqtt("nibe/operation_mode", "Heizungspumpe ist an, AMS aus") #power on, heatpump off
        elif reg28_value == 26634 and reg29_value == 49706 and reg30_value == 170:
            publish_mqtt("nibe/operation_mode", "Öljy paluu") #oil return
        elif reg28_value == 16394 and reg29_value == 49706 and reg30_value == 610:
            publish_mqtt ("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 16394 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt ("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 16650 and reg29_value == 49706 and reg30_value == 610:
            publish_mqtt ("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 0x0000 and reg29_value == 0xC22A and reg30_value == 0x003C:
            publish_mqtt("nibe/operation_mode", "Zusatzheizung an") #additional heating only
        elif reg28_value == 16385 and reg29_value == 41514 and reg30_value == 50:
            publish_mqtt("nibe/operation_mode", "Brauchwasser") #domestic water
        elif reg28_value == 16393 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Heizung") # heating
        elif reg28_value == 10 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Pois päältä") #power on, heatpump off
        elif reg28_value == 28 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 10 and reg29_value == 49706 and reg30_value == 610:
            publish_mqtt("nibe/operation_mode", "Pois päältä") #power on, heatpump off
        elif reg28_value == 32776 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Frostschutz") #freeze protection
        elif reg28_value == 32778 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Frostschutz") #freeze protection
        elif reg28_value == 17419 and reg29_value == 41514 and reg30_value == 510:
            publish_mqtt("nibe/operation_mode", "Brauchwasser") #domestic water
        elif reg28_value == 17425 and reg29_value == 41514 and reg30_value == 450:
            publish_mqtt("nibe/operation_mode", "extra Brauchwasser") #extra domestic water
        elif reg28_value == 24586 and reg29_value == 49706 and reg30_value == 270:
            publish_mqtt("nibe/operation_mode", "Enteisen") #defrost
        elif reg28_value == 24842 and reg29_value == 49706 and reg30_value == 270:
            publish_mqtt("nibe/operation_mode", "Enteisen") #defrost
        elif reg28_value == 17409 and reg29_value == 41514 and reg30_value == 30:
            publish_mqtt("nibe/operation_mode", "Brauchwasser") #domestic water
        else:
            logger.warning(f"Unknown combination of register values: reg28={reg28_value}, reg29={reg29_value}, reg30={reg30_value}")
            publish_mqtt("nibe/operation_mode", f"Unknown mode: reg28={reg28_value}, reg29={reg29_value}, reg30={reg30_value}")
        return None  # Avoid falling through to unhandled case

    # Handle additional heating using nibe/additional_heating_allowed
    if reg == 32:
        logger.debug(f"Register 32 (additional heating allowed) value: {value}")
        publish_mqtt("nibe/additional_heating_allowed", str(value))
        return None

    # Handle heating status using register 31
    if reg == 31:
        logger.debug(f"Register 31 (heating status) value: {value}")
        if value == 1:
            publish_mqtt("nibe/heating_status", "auto")
        elif value == 3:
            publish_mqtt("nibe/heating_status", "Heizung") #heating
        elif value == 5:
            publish_mqtt("nibe/heating_status", "Brauchwasser") #domestic water
        elif value == 6:
            publish_mqtt("nibe/heating_status", "Zusatzheizung") #additional heating
        return None

    # Handle temperature and flow registers
    if reg in [1, 5, 6, 7, 12, 23, 11, 13, 14, 15, 16, 17, 18, 21]:
        logger.debug(f"Register {reg} (temperature/flow) value: {value}")
        return float(unpack('h', pack('H', value))[0] / 10)

    # Handle general integer registers
    if reg in [0, 33, 34, 35, 36, 38, 44, 45, 46, 48, 100, 101, 102, 103, 104, 105]:
        logger.debug(f"Register {reg} (general integer) value: {value}")
        return int(value)

    # Handle signed values for heating curve and degree minutes
    if reg in [4, 8]:
        logger.debug(f"Register {reg} (signed value) value: {value}")
        return int(unpack('h', pack('H', value))[0] / 10)

    # Handle compressor starts
    if reg == 25:
        logger.debug(f"Register 25 (compressor starts) value: {value}")
        return int(value / 10)

    # Handle frequency, pressure, phase, and runtime registers
    if reg in [9, 10, 19, 22, 24]:
        logger.debug(f"Register {reg} (frequency/pressure) value: {value}")
        return float(value / 10)
        
    # Low pressure return value + 30    
    if reg == 20:
        logger.debug(f"Register {reg} (temperature/flow) value: {value}")
        if value > 100:
            return float((unpack('h', pack('H', value))[0] / 10) - 30)        
        else:
            return float(unpack('h', pack('H', value))[0] / 10)
    
    # Handle hysteresis and BW registers
    if reg in [40, 47]:
        logger.debug(f"Register {reg} (hysteresis/BW reg) value: {value}")
        return float(value / 2)

    # Handle stop temperature and domestic water temperatures
    if reg in [43, 49, 50]:
        logger.debug(f"Register {reg} (temperature) value: {value}")
        return float(value)

    # Handle 8b message temperatures
    if reg in [128, 129, 131, 135]:
        logger.debug(f"Register {reg} (temperature) value: {value}")
        return float(unpack('h', pack('H', value))[0] / 10)

    # Log unknown registers
    logger.warning(f"Register {reg} is not handled")
    return None

# Prüft, ob die erwartete Sequenz empfangen wurde
def wait_for_sequence():
    logger.info(f"Warte auf Sequenz: {TRIGGER_SEQUENCE.hex(' ')}")
    buffer = bytearray()

    while True:
        byte = ser.read(1)  # Einzelnes Byte lesen
        if not byte:
            continue

        buffer.append(byte[0])

        # Fenster auf Sequenzlänge begrenzen
        if len(buffer) > len(TRIGGER_SEQUENCE):
            buffer.pop(0)

        if bytes(buffer) == TRIGGER_SEQUENCE:
            logger.info("Erwartete Sequenz empfangen!")
            return True

# Serielle Nachricht senden
def send_serial_message():
    try:
        #if not ser.is_open:
            #ser.open()

        ser.write(SEND_MESSAGE)
        ser.flush()
        logger.info(f"Nachricht gesendet: {SEND_MESSAGE.hex(' ')}")
        # publish_mqtt("nibe/status", "Nachricht gesendet")
    except Exception as e:
        logger.error(f"Fehler beim Senden der Nachricht: {e}")

def run():
    logger.info("Starting the main loop...")

    # Publish availability as "online" at the start of the script
    publish_availability("online")

    # Publish discovery payloads for all sensors
    publish_discovery_payloads()

    mqtt_client.loop_start()
    try:
        while True:
            try:
                if ser.read(1)[0] != 0x03:
                    logger.debug("No start byte found")
                    continue
                logger.debug("Start byte found, reading data...")
                ret = ser.read(2)
                if ret[0] != 0x00 or (ret[1] not in [0x14, 0xf1]):
                    logger.debug("Invalid start frame")
                    continue
                if ret[1] == 0x14:
                    ser.write(b"\x06")
                    frm = ser.read(4)
                    if frm[0] == 0x03: 
                        continue
                    l = int(frm[3])
                    frm += ser.read(l + 1)
                    ser.write(b"\x06")
                    crc = 0
                    for i in frm[:-1]:
                        crc ^= i
                    if crc != frm[-1]:
                        logger.warning("Frame CRC error")
                        continue
                    msg = frm[4:-1]
                    l = len(msg)
                    i = 4
                    while i <= l:
                        reg = msg[i - 3]
                        if i != l and (msg[i] == 0x00 or i == (l - 1)):
                            raw = bytes([msg[i - 2], msg[i - 1]])
                            i += 4
                        else:
                            raw = bytes([msg[i - 2]])
                            i += 3
                        if reg in nibe_registers:
                            value = _decode(reg, raw)
                            if value is not None:
                                mqtt_topic = nibe_registers[reg]
                                publish_mqtt(mqtt_topic, value)
                #ret = ser.read(3)
                #elif ret[1] == 0xf1:
                if ret[1] == 0xf1:
                    ack = ser.read(1)
                    if ack[0] != 0x06:
                        continue
                    logger.debug("8888888888888888888888888888888888888888888888888888888888888888bbbbbbbbbbbbbbb")
                    frm = ser.read(4)
                    if frm[0] == 0x03:     #Falls das erste Byte der empfangenen Nachricht wieder 0x03 ist, wird sie ignoriert. Das passiert am Ende der Nachricht (03 00).
                        continue
                    l = int(frm[3])
                    frm += ser.read(l + 1)
                    crc = 0
                    for i in frm[:-1]:          
                        crc ^= i
                    if crc != frm[-1]:
                        logger.debug("Frame CRC error 8888888888888888888888888888888888888888888888888888bbbbbbbbbbbbbbb")
                        continue
                    msg = frm[4:-1]
                    l = len(msg)
                    i = 4
                    while i <= l:
                        reg = msg[i - 4]  # Fix: Korrekte Register-Indexierung
                        if i != l and (msg[i - 1] == 0x00 or i == (l - 1)):
                            #raw = bytes([msg[i - 3] // 16, msg[i - 2]])  # Fix: Richtige Byte-Kombination
                            raw = bytes([(msg[i - 3] >> 4) | (msg[i - 3] & 0x0F), msg[i - 2]])
                            i += 4
                        else:
                            raw = bytes([msg[i - 2]])
                            i += 3
                        if reg in nibe_registers:
                            value = _decode(reg, raw)
                            if value is not None:
                                mqtt_topic = nibe_registers[reg]
                                publish_mqtt(mqtt_topic, value)
                #else:
                    #logger.debug(f"Unbekannter Nachrichtentyp: {ret[1]}")
            except Exception as e:
                logger.warning(f"Error in Nibe data processing: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Script interrupted, shutting down...")
    finally:
        # Publish availability as "offline" when the script is stopped
        publish_availability("offline")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

if __name__ == "__main__":
    logger.info("Starting Nibe heat pump MQTT bridge")
    run()
mqtt_client.subscribe(MQTT_TOPIC_TRIGGER)
mqtt_client.loop_start()

# Register mapping, expanded to match original script
nibe_registers = {
    0: "nibe/cpu_id",
    1: "nibe/outdoor_temp_c",  
    4: "nibe/heating_curve",
    5: "nibe/flow_setpoint_c",  
    6: "nibe/flow_actual_c",  
    7: "nibe/return_temp_c",
    8: "nibe/degree_minutes",
    12: "nibe/domestic_hot_water_top_temp",
    25: "nibe/compressor_starts",
    31: "nibe/heating_status",
    32: "nibe/additional_heating_allowed",
    33: "nibe/max_df_compressor",
    34: "nibe/verd_freq_reg_p",
    35: "nibe/min_start_time_freq_min",
    36: "nibe/min_time_const_freq_min",
    38: "nibe/comp_freq_grad_min",
    44: "nibe/pump_speed_percent",
    45: "nibe/bw_reg_p",
    46: "nibe/bw_reg_q",
    48: "nibe/bw_reg_value_xp_percent",
    100: "nibe/date_year",
    101: "nibe/date_month",
    102: "nibe/date_day",
    103: "nibe/time_hour",
    104: "nibe/time_minute",
    105: "nibe/time_second",
    24: "nibe/run_time_compressor_h",
    9: "nibe/comp_freq_desired_hz",
    10: "nibe/comp_freq_actual_hz",
    19: "nibe/high_pressure_bar",
    20: "nibe/low_pressure_bar",
    22: "nibe/ams_phase_is_a",
    40: "nibe/hysteresis",
    47: "nibe/bw_reg_xp",
    43: "nibe/stop_temp_heating_c",
    49: "nibe/domestic_hot_water_start_temp",
    50: "nibe/domestic_hot_water_stop_temp",
    28: "nibe/operation_mode_reg_28",
    29: "nibe/operation_mode_reg_29",
    30: "nibe/operation_mode_reg_30",
    23: "nibe/inverter_temp_tho_ip",
    11: "nibe/condenser_off_max",
    13: "nibe/domestic_hot_water_bottom",
    14: "nibe/tho_r1_evap_temp",
    15: "nibe/tho_r2_evap_temp",
    16: "nibe/suction_gas_temp_tho_s",
    17: "nibe/hot_gas_temp_tho_d",
    18: "nibe/liquid_temp_ams",
    21: "nibe/resp_at_ams_tho_a",
    #############################
    128: "nibe/8b_temp1",
    129: "nibe/8b_temp2",
    131: "nibe/8b_temp3",
    135: "nibe/8b_temp4",
}
  

# Define unique IDs and MQTT discovery configurations for each sensor
mqtt_discovery_sensors = {
    "nibe/pump_speed_percent": {
        "name": "Pumpengeschwindigkeit",
        "unit_of_measurement": "%",
        "state_topic": "nibe/pump_speed_percent",
        "unique_id": "nibe_pump_speed_percent"
    },
#    "nibe/cpu_id": {
#        "name": "CPU ID",
#        "state_topic": "nibe/cpu_id",
#        "unique_id": "nibe_cpu_id"
#    },
    "nibe/inverter_temp_tho_ip": {
        "name": "Invertertemp. Tho-IP",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/inverter_temp_tho_ip",
        "unique_id": "nibe_inverter_temp_tho_ip"
    },
    "nibe/operation_mode": {
        "name": "Nibe Operation Mode",
        "state_topic": "nibe/operation_mode",
    "unique_id": "nibe_operation_mode"
    },
    "nibe/condenser_off_max": {
        "name": "Kondensator aus (MAX)",
        "state_topic": "nibe/condenser_off_max",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_condenser_off_max"
    },
    "nibe/domestic_hot_water_bottom": {
        "name": "Brauchwasser unten",
        "state_topic": "nibe/domestic_hot_water_bottom",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_domestic_hot_water_bottom"
    },
    "nibe/tho_r1_evap_temp": {
        "name": "Verd. Temp. Tho-R1",
        "state_topic": "nibe/tho_r1_evap_temp",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_tho_r1_evap_temp"
    },
    "nibe/tho_r2_evap_temp": {
        "name": "Verd. Temp. Tho-R2",
        "state_topic": "nibe/tho_r2_evap_temp",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_tho_r2_evap_temp"
    },
    "nibe/suction_gas_temp_tho_s": {
        "name": "Sauggastemp. Tho-S",
        "state_topic": "nibe/suction_gas_temp_tho_s",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_suction_gas_temp_tho_s"
    },
    "nibe/hot_gas_temp_tho_d": {
        "name": "Heissgastemp. Tho-D",
        "state_topic": "nibe/hot_gas_temp_tho_d",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_hot_gas_temp_tho_d"
    },
    "nibe/liquid_temp_ams": {
        "name": "Flüssigkeitstemperatur AMS",
        "state_topic": "nibe/liquid_temp_ams",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_liquid_temp_ams"
    },
    "nibe/resp_at_ams_tho_a": {
        "name": "Atemp. am AMS Tho-A",
        "state_topic": "nibe/resp_at_ams_tho_a",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_resp_at_ams_tho_a"
    },     
    "nibe/heating_curve": {
        "name": "Heizkurvenverschiebung",
        "state_topic": "nibe/heating_curve",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_heating_curve"
    },
    "nibe/return_temp_c": {
        "name": "Rücklauf",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/return_temp_c",
        "unique_id": "nibe_return_temp_c"
    },
    "nibe/degree_minutes": {
        "name": "Gradminuten",
        "state_topic": "nibe/degree_minutes",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_degree_minutes"
    },
    "nibe/domestic_hot_water_top_temp": {
        "name": "Brauchwasser oben",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/domestic_hot_water_top_temp",
        "unique_id": "nibe_domestic_hot_water_top_temp"
    },
    "nibe/max_df_compressor": {
        "name": "Max dF Verdichter",
        "state_topic": "nibe/max_df_compressor",
        "unit_of_measurement": "Hz",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_max_df_compressor"
    },
    "nibe/verd_freq_reg_p": {
        "name": "Verd. Freq. reg P",
        "state_topic": "nibe/verd_freq_reg_p",
        "unit_of_measurement": "Hz",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_verd_freq_reg_p"
    },
    "nibe/min_start_time_freq_min": {
        "name": "Min Startzeit Freq",
        "unit_of_measurement": "Hz",
        "state_topic": "nibe/min_start_time_freq_min",
        "unique_id": "nibe_min_start_time_freq_min"
    },
    "nibe/min_time_const_freq_min": {
        "name": "Minzeit konst. Freq",
        "state_topic": "nibe/min_time_const_freq_min",
        "unit_of_measurement": "Hz",
        "unique_id": "nibe_min_time_const_freq_min"
    },
    "nibe/comp_freq_grad_min": {
        "name": "Verd. Freq. GradMin",
        "state_topic": "nibe/comp_freq_grad_min",
        "unit_of_measurement": "Hz",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_comp_freq_grad_min"
    },
    "nibe/bw_reg_p": {
        "name": "Bw reg P",
        "state_topic": "nibe/bw_reg_p",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_bw_reg_p"
    },
    "nibe/bw_reg_q": {
        "name": "Bw reg Q",
        "state_topic": "nibe/bw_reg_q",
        "value_template": "{{ value | float }}",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "unique_id": "nibe_bw_reg_q"
    },
    "nibe/bw_reg_value_xp_percent": {
        "name": "Bw reg Wert xP",
        "unit_of_measurement": "%",
        "state_topic": "nibe/bw_reg_value_xp_percent",
        "unique_id": "nibe_bw_reg_value_xp_percent"
    },

#enable if you want mqtt to publish time and date
#    "nibe/date_year": {
#        "name": "Date Year",
#        "state_topic": "nibe/date_year",
#        "unique_id": "nibe_date_year"
#    },
#    "nibe/date_month": {
#        "name": "Date Month",
#        "state_topic": "nibe/date_month",
#        "unique_id": "nibe_date_month"
#    },
#    "nibe/date_day": {
#        "name": "Date Day",
#        "state_topic": "nibe/date_day",
#        "unique_id": "nibe_date_day"
#    },
#    "nibe/time_hour": {
#        "name": "Time Hour",
#        "state_topic": "nibe/time_hour",
#        "unique_id": "nibe_time_hour"
#    },
#    "nibe/time_minute": {
#        "name": "Time Minute",
#        "state_topic": "nibe/time_minute",
#        "unique_id": "nibe_time_minute"
#    },
#    "nibe/time_second": {
#        "name": "Time Second",
#        "state_topic": "nibe/time_second",
#        "unique_id": "nibe_time_second"
#    },
    "nibe/run_time_compressor_h": {
        "name": "Verdichterlaufzeit",
        "state_topic": "nibe/run_time_compressor_h",
        "device_class": "duration",
        "unit_of_measurement": "h",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_run_time_compressor_h"
    },
    "nibe/comp_freq_desired_hz": {
        "name": "Verd. Freq. Soll",
        "unit_of_measurement": "Hz",
        "state_topic": "nibe/comp_freq_desired_hz",
        "unique_id": "nibe_comp_freq_desired_hz"
    },
    "nibe/comp_freq_actual_hz": {
        "name": "Verd. Freq. Ist",
        "unit_of_measurement": "Hz",
        "state_topic": "nibe/comp_freq_actual_hz",
        "unique_id": "nibe_comp_freq_actual_hz"
    },
    "nibe/high_pressure_bar": {
        "name": "Hochdruck",
        "unit_of_measurement": "bar",
        "state_topic": "nibe/high_pressure_bar",
        "unique_id": "nibe_high_pressure_bar"
    },
    "nibe/low_pressure_bar": {
        "name": "Niederdruck",
        "unit_of_measurement": "bar",
        "state_topic": "nibe/low_pressure_bar",
        "unique_id": "nibe_low_pressure_bar"
    },
    "nibe/ams_phase_is_a": {
        "name": "AMS Phase Ist",
        "unit_of_measurement": "A",
        "state_topic": "nibe/ams_phase_is_a",
        "unique_id": "nibe_ams_phase_is_a"
    },
    "nibe/hysteresis": {
        "name": "Hystere",
        "state_topic": "nibe/hysteresis",
        "state_class": "measurement",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_hysteresis"
    },
    "nibe/bw_reg_xp": {
        "name": "Bw reg xP",
        "state_topic": "nibe/bw_reg_xp",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_bw_reg_xp"
    },
    "nibe/stop_temp_heating_c": {
        "name": "Stopptemperatur Heizen",
        "unit_of_measurement": "°C",
        "state_topic": "nibe/stop_temp_heating_c",
        "unique_id": "nibe_stop_temp_heating_c"
    },
    "nibe/domestic_hot_water_start_temp": {
        "name": "Brauchwasser Starttemperatur",
        "unit_of_measurement": "°C",
        "state_topic": "nibe/domestic_hot_water_start_temp",
        "unique_id": "nibe_domestic_hot_water_start_temp"
    },
    "nibe/domestic_hot_water_stop_temp": {
        "name": "Brauchwasser Stoptemperatur",
        "unit_of_measurement": "°C",
        "state_topic": "nibe/domestic_hot_water_stop_temp",
        "unique_id": "nibe_domestic_hot_water_stop_temp"
    },
    "nibe/outdoor_temp_c": {
        "name": "Außentemperatur",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/outdoor_temp_c",
        "unique_id": "nibe_outdoor_temp_c"
    },
    "nibe/flow_setpoint_c": {
        "name": "Vorlauf Soll",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/flow_setpoint_c",
        "unique_id": "nibe_flow_setpoint_c"
    },
    "nibe/flow_actual_c": {
        "name": "Vorlauf Ist",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/flow_actual_c",
        "unique_id": "nibe_flow_actual_c"
    },
    "nibe/compressor_starts": {
        "name": "Verdichterstarts",
        "state_topic": "nibe/compressor_starts",
        "unit_of_measurement": "x",
        "state_class": "total_increasing",
        "value_template": "{{ value | float }}",
        "unique_id": "nibe_compressor_starts"
    },
    "nibe/heating_status": {
        "name": "Status Heizung",
        "state_topic": "nibe/heating_status",
        "unique_id": "nibe_heating_status"
    },
    ######################################################
    "nibe/8b_temp1": {
        "name": "8b Temp1",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp1",
        "unique_id": "nibe_8b_temp1"
    },
    "nibe/8b_temp2": {
        "name": "8b Temp2",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp2",
        "unique_id": "nibe_8b_temp2"
    },
    "nibe/8b_temp3": {
        "name": "8b Temp3",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp3",
        "unique_id": "nibe_8b_temp3"
    },
    "nibe/8b_temp4": {
        "name": "8b Temp4",
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_topic": "nibe/8b_temp4",
        "unique_id": "nibe_8b_temp4"
    },
######################################################
    "nibe/additional_heating": {
        "name": "Zusatzheizung erlaubt",
        "state_topic": "nibe/additional_heating_allowed",
        "unique_id": "nibe_additional_heating_allowed",
        "value_template": "{{ value | int }}",  # Ensure Home Assistant interprets 0 and 1 as integers
        "availability_topic": "nibe/status",
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": {
            "identifiers": ["nibe_heat_pump"],
            "name": "Nibe Wärmepumpe",
            "manufacturer": "Nibe",
            "model": "ACVM 270",
            "sw_version": "1.0"
    }
},

}


# Publish MQTT discovery payload for each sensor
def publish_discovery_payloads():
    for sensor, config in mqtt_discovery_sensors.items():
        discovery_topic = f"homeassistant/sensor/{config['unique_id']}/config"
        
        # Build the base payload
        payload = {
            "name": config["name"],
            "state_topic": config["state_topic"],
            "unique_id": config["unique_id"],
            "availability_topic": "nibe/status",
            "payload_available": "online",
            "payload_not_available": "offline",
            "device": {
                "identifiers": ["nibe_heat_pump"],
                "name": "Nibe ACVM 270 Split",
                "manufacturer": "Nibe",
                "model": "ACVM 270",
                "sw_version": "1.0"
            }
        }

        # Add optional fields only if they exist
        if config.get("unit_of_measurement"):
            payload["unit_of_measurement"] = config["unit_of_measurement"]
        if config.get("device_class"):
            payload["device_class"] = config["device_class"]
        if config.get("value_template"):
            payload["value_template"] = config["value_template"]

        # Publish the discovery payload as JSON
        mqtt_client.publish(discovery_topic, str(payload).replace("'", '"'), qos=0, retain=True)
        logger.info(f"Published MQTT discovery payload for {config['name']}")

# Initialize global variables to store register 28, 29, and 30 values
reg28_value = None
reg29_value = None
reg30_value = None

def _decode(reg, raw):
    global reg28_value, reg29_value, reg30_value  # Ensure these variables are accessible
    
    if len(raw) == 2:
        value = unpack('>H', raw)[0]
    else:
        value = unpack('B', raw)[0]

    # Handle registers 28, 29, and 30
    if reg == 28:
        reg28_value = value
        logger.debug(f"Register 28 (operation mode) value: {reg28_value}")
        return None  # Avoid falling through to unhandled case
    elif reg == 29:
        reg29_value = value
        logger.debug(f"Register 29 (operation mode) value: {reg29_value}")
        return None  # Avoid falling through to unhandled case
    elif reg == 30:
        reg30_value = value
        logger.debug(f"Register 30 (operation mode) value: {reg30_value}")

        # Now that we have all three registers, interpret the state
        if reg28_value == 0x0000 and reg29_value == 0x8222 and reg30_value == 0x0032:
            publish_mqtt("nibe/operation_mode", "Standby") #pwer on, heatpump off
        elif reg28_value == 0x4409 and reg29_value == 0xA22A and reg30_value == 0x01FE:
            publish_mqtt("nibe/operation_mode", "Brauchwasserbereitung, AMS an") #domestic water
        elif reg28_value == 0x0008 and reg29_value == 0xC22A and reg30_value == 0x000A:
            publish_mqtt("nibe/operation_mode", "Heizungspumpe ist an, AMS aus") #power on, heatpump off
        elif reg28_value == 26634 and reg29_value == 49706 and reg30_value == 170:
            publish_mqtt("nibe/operation_mode", "Öljy paluu") #oil return
        elif reg28_value == 16394 and reg29_value == 49706 and reg30_value == 610:
            publish_mqtt ("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 16394 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt ("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 16650 and reg29_value == 49706 and reg30_value == 610:
            publish_mqtt ("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 0x0000 and reg29_value == 0xC22A and reg30_value == 0x003C:
            publish_mqtt("nibe/operation_mode", "Zusatzheizung an") #additional heating only
        elif reg28_value == 16385 and reg29_value == 41514 and reg30_value == 50:
            publish_mqtt("nibe/operation_mode", "Brauchwasser") #domestic water
        elif reg28_value == 16393 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Heizung") # heating
        elif reg28_value == 10 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Pois päältä") #power on, heatpump off
        elif reg28_value == 28 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Heizung") #heating
        elif reg28_value == 10 and reg29_value == 49706 and reg30_value == 610:
            publish_mqtt("nibe/operation_mode", "Pois päältä") #power on, heatpump off
        elif reg28_value == 32776 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Frostschutz") #freeze protection
        elif reg28_value == 32778 and reg29_value == 49706 and reg30_value == 10:
            publish_mqtt("nibe/operation_mode", "Frostschutz") #freeze protection
        elif reg28_value == 17419 and reg29_value == 41514 and reg30_value == 510:
            publish_mqtt("nibe/operation_mode", "Brauchwasser") #domestic water
        elif reg28_value == 17425 and reg29_value == 41514 and reg30_value == 450:
            publish_mqtt("nibe/operation_mode", "extra Brauchwasser") #extra domestic water
        elif reg28_value == 24586 and reg29_value == 49706 and reg30_value == 270:
            publish_mqtt("nibe/operation_mode", "Enteisen") #defrost
        elif reg28_value == 24842 and reg29_value == 49706 and reg30_value == 270:
            publish_mqtt("nibe/operation_mode", "Enteisen") #defrost
        elif reg28_value == 17409 and reg29_value == 41514 and reg30_value == 30:
            publish_mqtt("nibe/operation_mode", "Brauchwasser") #domestic water
        else:
            logger.warning(f"Unknown combination of register values: reg28={reg28_value}, reg29={reg29_value}, reg30={reg30_value}")
            publish_mqtt("nibe/operation_mode", f"Unknown mode: reg28={reg28_value}, reg29={reg29_value}, reg30={reg30_value}")
        return None  # Avoid falling through to unhandled case

    # Handle additional heating using nibe/additional_heating_allowed
    if reg == 32:
        logger.debug(f"Register 32 (additional heating allowed) value: {value}")
        publish_mqtt("nibe/additional_heating_allowed", str(value))
        return None

    # Handle heating status using register 31
    if reg == 31:
        logger.debug(f"Register 31 (heating status) value: {value}")
        if value == 1:
            publish_mqtt("nibe/heating_status", "auto")
        elif value == 3:
            publish_mqtt("nibe/heating_status", "Heizung") #heating
        elif value == 5:
            publish_mqtt("nibe/heating_status", "Brauchwasser") #domestic water
        elif value == 6:
            publish_mqtt("nibe/heating_status", "Zusatzheizung") #additional heating
        return None

    # Handle temperature and flow registers
    if reg in [1, 5, 6, 7, 12, 23, 11, 13, 14, 15, 16, 17, 18, 21]:
        logger.debug(f"Register {reg} (temperature/flow) value: {value}")
        return float(unpack('h', pack('H', value))[0] / 10)

    # Handle general integer registers
    if reg in [0, 33, 34, 35, 36, 38, 44, 45, 46, 48, 100, 101, 102, 103, 104, 105]:
        logger.debug(f"Register {reg} (general integer) value: {value}")
        return int(value)

    # Handle signed values for heating curve and degree minutes
    if reg in [4, 8]:
        logger.debug(f"Register {reg} (signed value) value: {value}")
        return int(unpack('h', pack('H', value))[0] / 10)

    # Handle compressor starts
    if reg == 25:
        logger.debug(f"Register 25 (compressor starts) value: {value}")
        return int(value / 10)

    # Handle frequency, pressure, phase, and runtime registers
    if reg in [9, 10, 19, 22, 24]:
        logger.debug(f"Register {reg} (frequency/pressure) value: {value}")
        return float(value / 10)
        
    # Low pressure return value + 30    
    if reg == 20:
        logger.debug(f"Register {reg} (temperature/flow) value: {value}")
        if value > 100:
            return float((unpack('h', pack('H', value))[0] / 10) - 30)        
        else:
            return float(unpack('h', pack('H', value))[0] / 10)
    
    # Handle hysteresis and BW registers
    if reg in [40, 47]:
        logger.debug(f"Register {reg} (hysteresis/BW reg) value: {value}")
        return float(value / 2)

    # Handle stop temperature and domestic water temperatures
    if reg in [43, 49, 50]:
        logger.debug(f"Register {reg} (temperature) value: {value}")
        return float(value)

    # Handle 8b message temperatures
    if reg in [128, 129, 131, 135]:
        logger.debug(f"Register {reg} (temperature) value: {value}")
        return float(unpack('h', pack('H', value))[0] / 10)

    # Log unknown registers
    logger.warning(f"Register {reg} is not handled")
    return None

# Prüft, ob die erwartete Sequenz empfangen wurde
def wait_for_sequence():
    logger.info(f"Warte auf Sequenz: {TRIGGER_SEQUENCE.hex(' ')}")
    buffer = bytearray()

    while True:
        byte = ser.read(1)  # Einzelnes Byte lesen
        if not byte:
            continue

        buffer.append(byte[0])

        # Fenster auf Sequenzlänge begrenzen
        if len(buffer) > len(TRIGGER_SEQUENCE):
            buffer.pop(0)

        if bytes(buffer) == TRIGGER_SEQUENCE:
            logger.info("Erwartete Sequenz empfangen!")
            return True

# Serielle Nachricht senden
def send_serial_message():
    try:
        if not ser.is_open:
            ser.open()

        ser.write(SEND_MESSAGE)
        ser.flush()
        logger.info(f"Nachricht gesendet: {SEND_MESSAGE.hex(' ')}")
        # publish_mqtt("nibe/status", "Nachricht gesendet")
    except Exception as e:
        logger.error(f"Fehler beim Senden der Nachricht: {e}")

def run():
    logger.info("Starting the main loop...")

    # Publish availability as "online" at the start of the script
    publish_availability("online")

    # Publish discovery payloads for all sensors
    publish_discovery_payloads()

    mqtt_client.loop_start()
    try:
        while True:
            try:
                if ser.read(1)[0] != 0x03:
                    logger.debug("No start byte found")
                    continue
                logger.debug("Start byte found, reading data...")
                ret = ser.read(2)
                if ret[0] != 0x00 or (ret[1] not in [0x14, 0xf1]):
                    logger.debug("Invalid start frame")
                    continue
                if ret[1] == 0x14:
                    ser.write(b"\x06")
                    frm = ser.read(4)
                    if frm[0] == 0x03: 
                        continue
                    l = int(frm[3])
                    frm += ser.read(l + 1)
                    ser.write(b"\x06")
                    crc = 0
                    for i in frm[:-1]:
                        crc ^= i
                    if crc != frm[-1]:
                        logger.warning("Frame CRC error")
                        continue
                    msg = frm[4:-1]
                    l = len(msg)
                    i = 4
                    while i <= l:
                        reg = msg[i - 3]
                        if i != l and (msg[i] == 0x00 or i == (l - 1)):
                            raw = bytes([msg[i - 2], msg[i - 1]])
                            i += 4
                        else:
                            raw = bytes([msg[i - 2]])
                            i += 3
                        if reg in nibe_registers:
                            value = _decode(reg, raw)
                            if value is not None:
                                mqtt_topic = nibe_registers[reg]
                                publish_mqtt(mqtt_topic, value)
                #ret = ser.read(3)
                #elif ret[1] == 0xf1:
                if ret[1] == 0xf1:
                    ack = ser.read(1)
                    if ack[0] != 0x06:
                        continue
                    logger.debug("8888888888888888888888888888888888888888888888888888888888888888bbbbbbbbbbbbbbb")
                    frm = ser.read(4)
                    if frm[0] == 0x03:     #Falls das erste Byte der empfangenen Nachricht wieder 0x03 ist, wird sie ignoriert. Das passiert am Ende der Nachricht (03 00).
                        continue
                    l = int(frm[3])
                    frm += ser.read(l + 1)
                    crc = 0
                    for i in frm[:-1]:          
                        crc ^= i
                    if crc != frm[-1]:
                        logger.debug("Frame CRC error 8888888888888888888888888888888888888888888888888888bbbbbbbbbbbbbbb")
                        continue
                    msg = frm[4:-1]
                    l = len(msg)
                    i = 4
                    while i <= l:
                        reg = msg[i - 4]  # Fix: Korrekte Register-Indexierung
                        if i != l and (msg[i - 1] == 0x00 or i == (l - 1)):
                            #raw = bytes([msg[i - 3] // 16, msg[i - 2]])  # Fix: Richtige Byte-Kombination
                            raw = bytes([(msg[i - 3] >> 4) | (msg[i - 3] & 0x0F), msg[i - 2]])
                            i += 4
                        else:
                            raw = bytes([msg[i - 2]])
                            i += 3
                        if reg in nibe_registers:
                            value = _decode(reg, raw)
                            if value is not None:
                                mqtt_topic = nibe_registers[reg]
                                publish_mqtt(mqtt_topic, value)
                #else:
                    #logger.debug(f"Unbekannter Nachrichtentyp: {ret[1]}")
            except Exception as e:
                logger.warning(f"Error in Nibe data processing: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Script interrupted, shutting down...")
    finally:
        # Publish availability as "offline" when the script is stopped
        publish_availability("offline")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

if __name__ == "__main__":
    logger.info("Starting Nibe heat pump MQTT bridge")
    run()
