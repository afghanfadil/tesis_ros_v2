import time
import rospy
from marvelmind_nav.msg import hedge_pos_a
import numpy as np
# untuk menulis csv
import csv

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
                xbef = 0,
                xnow = 0,
                ybef = 0,
                ynow = 0,
                t_bef = 0,
                t_now = 0 )

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

def speed_calculation(msg): 
    params.t_now = time.time()
    params.xnow = msg.x_m
    params.ynow = msg.y_m
    distance = np.sqrt((params.xnow-params.xbef)**2+(params.ynow-params.ybef)**2)
    delta_time = params.t_now - params.t_bef
    speed = distance/delta_time

    params.xbef = params.xnow
    params.ybef = params.ynow
    params.t_bef = params.t_now

    backup = [params.t_now, speed]

    with open('speed_200.csv', 'a') as f:
            writer = csv.writer(f, delimiter=',')
            writer.writerow(backup)

    print(speed)

def save_speed_value(client, userdata, message): 
    speed = float(message.payload.decode("utf-8"))
    params.t_now = time.time()
    backup = [params.t_now, speed]
    with open('speed_150.csv', 'a') as f:
            writer = csv.writer(f, delimiter=',')
            writer.writerow(backup)

params = get_params()
params.t_bef = time.time()
rospy.init_node('speed_callibration')
client = connect_mqtt()
client.subscribe(topic)
client.message_callback_add("/EV3_movement/speed_AV", save_speed_value)
client.loop_start()
# sub = rospy.Subscriber('/hedge_pos_a', hedge_pos_a, speed_calculation)

rospy.spin()