from paho.mqtt.client import Client
import socket
import json
from time import sleep

# -------------------------- Configuration -------------------------- #

MQTT_CLIENT_ID = "sailtrack-sensor-strain-cal"
CONFIG_FILE_PATH = "../rootfs/etc/sailtrack/sailtrack.conf"

# ------------------------------------------------------------------- #
mqtt = Client(MQTT_CLIENT_ID)
mqtt.username_pw_set("mosquitto", "sailtrack")
 
try:
    mqtt.connect("192.168.42.1")
    mqtt.loop_start()
    print("Right strain calibration:")
    shroud_load = input("Insert shroud tension in Kg:")
    detach_time = input("Insert train attach operation time:")
    mqtt.publish("sensor/strainRight/calibration", json.dumps({
    "calibrationLoad": shroud_load,
    "calibrationnScaleDelay": detach_time
    }))
    print("Calibration procedure on: setting zero tension")
    sleep(int(detach_time))
    print("Attach the strain to the shroud")
    sleep(int(detach_time))
    print("Right strain calibration terminated")
    print("Left strain calibration:")
    shroud_load = input("Insert shroud tension in Kg:")
    detach_time = input("Insert train detach operation time:")
    mqtt.publish("sensor/strainLeft/calibration", json.dumps({
    "calibrationLoad": shroud_load,
    "calibrationnScaleDelay": detach_time
    }))
    print("Calibration procedure on: setting zero tension")
    sleep(int(detach_time))
    print("Attach the strain to the shroud")
    sleep(int(detach_time))
    print("Right strain calibration terminated")

except socket.timeout:
    input("Connection timed out. Connect to SailTrack-Net and try again")


