#=========Feedback Steer Autonomous Truck Trailer MQTT Script==============
#Lab ICoDeS, Teknik Fisika, ITB
#Husnul Amri
#===========================================================================

#for used in master PC (with ROS)
# using MQTT
import paho.mqtt.client as mqtt
import numpy as np

# MQTT Stuff
broker ="192.168.1.101"
port = 1883
topic = "/EV3_movement/#"

# declare constant/initial parameter
def get_params():
    class Bunch:
        def __init__(self, **kwds):
            self.__dict__.update(kwds)
    # Declare constant parameters
    params = Bunch(
                speed = 0,
                steer = 0 )

    return params

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
    params.speed = message.payload.decode("utf-8")
    params.speed = int(params.speed)
    client.publish("/EV3_movement/speed_actuate",params.speed)

def steer_callback(client, userdata, message):
    params.steer = message.payload.decode("utf-8")
    params.steer = float(params.steer)
    # convert from degree to radian
    params.steer = params.steer * (np.pi/180)
    print("Steer Command:" + str(params.steer))

def steer_angle_callback(client, userdata, message):
    ADC = message.payload.decode("utf-8")
    ADC = int(ADC)
    # calculate ADC value based on desired steering angle
    ADC_target = 433.651 * params.steer + 472.593
    ADC_target = int(ADC_target)  

    if ADC >= (ADC_target-10) and ADC <= (ADC_target + 10): #tolerance, avoid jittering
        cs_steer = 0
    elif ADC > ADC_target: #perintah belok kanan
        cs_steer = -1
    elif ADC < ADC_target: #perintah belok kiri
        cs_steer = 1
    # else:   #maintain
    #     cs_steer = 0

    print("ADC_now: "+str(ADC)+" ADC_target: "+str(ADC_target)+" command_steer: "+str(cs_steer))
    client.publish("/EV3_movement/steer_actuate",cs_steer)


params = get_params()
client = connect_mqtt()
client.subscribe(topic)
client.message_callback_add("/EV3_movement/speed_command", speed_callback)
client.message_callback_add("/EV3_movement/steer_command", steer_callback)
client.message_callback_add("/EV3_movement/steer_angle_ADC", steer_angle_callback)
client.loop_forever()