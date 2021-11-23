#! /usr/bin/python3
import time
import rospy
from marvelmind_nav.msg import hedge_pos_a
from marvelmind_nav.msg import hedge_imu_fusion
from thesis_afghan.msg import State_Estimator
import numpy as np

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
                xtbef = 2.68,
                xtnow = 2.68,
                yhbef = -0.72,
                yhnow = -0.72,
                yawbef_t = 3.14,
                yawnow_t = 3.14,
                yawbef_h = 3.14,
                yawnow_h = 3.14,
                v_now = 0 )

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

# def speed_callback(client, userdata, message):
#     params.v_now= float(message.payload.decode("utf-8"))
#     print(params.v_now)

def save_speed_value(client, userdata, message): 
    speed = float(message.payload.decode("utf-8"))
    speed = (speed * 2.1)/100
    params.v_now = -speed

def to_euler(msg):
    x = msg.qx
    y = msg.qy
    z = msg.qz
    w = msg.qw    
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 (y**2 + z**2))
    
    return yaw

def vehicle_state(msg):
    if params.v_now > 0: #vehicle move
        # INi ganti pake yg di IMU
        if (msg.address) == '6':
            params.xnow_h = msg.xh_m
            params.ynow_h = msg.yh_m
            dist = np.sqrt((params.ynow_h-params.yhbef)**2-(params.xnow_h-params.xhbef)**2)
            if dist < 0.1:
                params.yaw_h = to_euler(msg.qx, msg.qy, msg.qz, msg.qw)                          
        elif (msg.address) == '8':
            params.xnow_t = msg.xt_m
            params.ynow_t = msg.yt_m
            dist = np.sqrt((params.ynow_t-params.ytbef)**2-(params.xnow_t-params.xtbef)**2)
            if dist < 0.1:
                params.yaw_t = to_euler(msg.qx, msg.qy, msg.qz, msg.qw)
    #print(params.xnow_h, params.ynow_h, params.yaw_h, params.xnow_t, params.ynow_t, params.yaw_t)    
        #params.xnow = msg.x_m
        #params.ynow = msg.y_m
        #dist = np.sqrt((params.ynow-params.ybef)**2-(params.xnow-params.xbef)**2)

        #if dist>0.1 : #its impossible to do 10cm displacement instantly
        #    params.xnow = params.xbef
        #    params.ynow = params.ybef

        #if (abs(params.xnow - params.xbef) > 0.005) or (abs(params.ynow - params.ybef) > 0.005) :  #only update yaw under this condition
        #    params.yaw = np.arctan2((params.ynow-params.ybef),(params.xnow-params.xbef))
        #    params.xbef = params.xnow
        #    params.ybef = params.ynow

        params.xtbef = params.xnow_t
        params.ytbef = params.ynow_t
        params.yawbef_t = params.yawnow_t
        params.xhbef = params.xnow_h
        params.yhbef = params.ynow_h
        params.yawbef_h = params.yawnow_h

        # print(params.xnow, params.ynow, params.yaw, params.v_now)
    # else : 
        # print(params.xnow, params.ynow, params.yaw, params.v_now)
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.yaw_t_est = params.yawnow_t
    pub_msg.xt_est = params.xnow_t
    pub_msg.yt_est = params.ynow_t
    pub_msg.yaw_h_est = params.yawnow_h
    pub_msg.xh_est = params.xnow_h
    pub_msg.yh_est = params.ynow_h
    pub_msg.v_est = params.v_now
    pub.publish(pub_msg)

params = get_params()
client = connect_mqtt()
client.subscribe(topic)
# client.message_callback_add("/EV3_movement/speed_command", speed_callback)
client.message_callback_add("/EV3_movement/speed_AV", save_speed_value)
client.loop_start()

rospy.init_node('vehicle_state')

freq = 50 # Hz

pub_msg = State_Estimator()
pub_msg.header.frame_id = 'vehicle_state'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

#ini nanti ganti pake yg IMU
sub = rospy.Subscriber('/hedge_pos_a', hedge_pos_a, vehicle_state) 
sub2 = rospy.Subscriber('/hedge_imu_fusion', hedge_imu_fusion, vehicle_state)

pub = rospy.Publisher('/vehicle_state', State_Estimator, queue_size=1)
rate = rospy.Rate(freq) # Hz

rospy.spin()
# client.loop_forever()
while True : 
    # print("Lets Rock")
    rate.sleep()
