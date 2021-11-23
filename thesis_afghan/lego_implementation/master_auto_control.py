#=========Feedback Steer Autonomous Truck Trailer MQTT Script==============
#Lab ICoDeS, Teknik Fisika, ITB
#Husnul Amri
#===========================================================================

#for used in master PC (with ROS)
# using MQTT
import paho.mqtt.client as mqtt
import numpy as np
import time

# MQTT Stuff
broker ="192.168.1.101"
port = 1883
topic = "/EV3_movement/#"

speed = 0
steer = 0
cs_lat_bef = 0
ADC_now = 0

# declare constant/initial parameter
# def get_params():
#     class Bunch:
#         def __init__(self, **kwds):
#             self.__dict__.update(kwds)
#     # Declare constant parameters
#     params = Bunch(
#                 speed = 0,
#                 steer = 0,
#                 cs_lat_bef = 0,
#                 ADC_now = 0 )

#     return params

def connect_mqtt() -> mqtt:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt.Client()
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def speed_callback(client, userdata, message):
    global speed

    speed = float(message.payload.decode("utf-8"))
    # cs_long = int(2729.123 * speed - 0.014995)
    # # cs_long = int(params.speed)
    # if speed==0:
    #     cs_long = 0
    # client.publish("/EV3_movement/speed_actuate",cs_long)
    # print("command_speed :"+str(cs_long))

def steer_callback(client, userdata, message):
    global steer

    steer = float(message.payload.decode("utf-8"))
    # convert from degree to radian
    steer = steer * (np.pi/180)
    # print("Steer Command:" + str(steer))

def steer_rad_callback(client, userdata, message):
    global steer

    steer = message.payload.decode("utf-8")
    steer = float(steer)
    if np.isnan(steer):
        steer = 0
    # print("Steer Command rad:" + str(steer))

def steer_angle_callback(client, userdata, message):
    global ADC_now

    ADC_now = int(message.payload.decode("utf-8"))
    # print("ADC Updated!")
    # ADC = int(ADC)
    # calculate ADC value based on desired steering angle
    
# print("ADC_now: "+str(ADC)+" ADC_target: "+str(ADC_target)+" command_steer: "+str(cs_lat))


client = connect_mqtt()
client.subscribe(topic)
client.message_callback_add("/EV3_movement/speed_command", speed_callback)
client.message_callback_add("/EV3_movement/steer_command_rad", steer_rad_callback)
client.message_callback_add("/EV3_movement/steer_command", steer_callback)
client.message_callback_add("/EV3_movement/steer_angle_ADC", steer_angle_callback)
client.loop_start()

while True:
    ADC_target = 433.651 * steer + 472.593
    ADC_target = int(ADC_target)  

    if ADC_now >= (ADC_target-10) and ADC_now <= (ADC_target + 10): #tolerance, avoid jittering
        cs_lat = 0
    elif ADC_now > ADC_target: #perintah belok kanan
        cs_lat = -1
    elif ADC_now < ADC_target: #perintah belok kiri
        cs_lat = 1
    
    if cs_lat != cs_lat_bef:         #update command only when the value changed
        client.publish("/EV3_command/steer_actuate",cs_lat,qos=1)
        cs_lat_bef = cs_lat
        print(cs_lat_bef, cs_lat, ADC_now, ADC_target, steer, speed)
    
    cs_long = int(2729.123 * speed - 0.014995)
    # cs_long = int(params.speed)
    if speed==0:
        cs_long = 0
    client.publish("/EV3_command/speed_actuate",cs_long,qos=0)

    time.sleep(1/20)