# Game Theory Planning
# Husnul Amri (2021)
# github : @moezeus
# adapted from RobustDecisionMaking (Gokul)

import numpy as np
import time
import math
import decimal
from ev3dev.ev3 import *
#from thesis.msg import Autonomous_Game
import rospy
from thesis_afghan.msg import lyapunov_jackknife
from thesis_afghan.msg import State_Estimator
# import rospy
# import os

#for used in master PC (with ROS)
# using MQTT
import paho.mqtt.client as mqtt
import numpy as np

# MQTT Stuff
broker ="192.168.1.101"
port = 1883
topic = "/EV3_movement/#"

# ROS thing
rospy.init_node('controller')
freq = 20 # Hz
pub = rospy.Publisher('/jackknife_control', lyapunov_jackknife, queue_size=1)
rate = rospy.Rate(freq) # Hz
pub_msg = lyapunov_jackknife()
pub_msg.header.frame_id = 'Lyapunov_Control'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

dt_sim = 1/freq

#Parameter Lyapunov Anti-Jackknife Control
xoffset = -36           #imu xoffset trailler in Mag
yoffset = 15.5          #imu yoffset trailler in Mag

lh = 0.145              #length of head truck (m)
lt = 0.313              #length of trailer (m) 
rb = 0.6                #Turning Radius (m)
vh_max = 0.2*1100/63    #Maksimum value of head velocity (m/s)
vh1 = vh_max*0.2        #Head velocity (m/s)
Qh = 0                  #the angle of head truck to x axis (rad)
Qt = 0                  #the angle of trailer to x axis in (rad)
Qs = 0                  #steering angle (rad)
Qd = 0                  #desired angle (rad)
dQs = 0                 #Lyapunov steering angle (degree)
Qh_e = 0                #error of head truck angle (to desired angle)
Qt_e = 0                #error of trailer angle (to desired angle)
Qht_e = 0               #error of angle between head and trailer 
h_eh = 0                #error distance of head (to desired path)
h_et = 0                #error distance of trailer (to desired path)

kh_eh = 1               #Coefficient of head distance error
kQh_e = 1               #Coefficient of head angle error
kh_et= 1                #Coefficient of trailer distance error
kQt_e = 1               #Coefficient of trailer angle error

print("Waiting.....")
time.sleep(5) #wait for live plot node (start it manually)
print("Simulation Start!")


# function definition
def connect_mqtt() -> mqtt:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def sqrt(n):
    assert n > 0
    with decimal.localcontext() as ctx:
        ctx.prec += 2 # increase precision to minimize round off error
        x, prior = decimal.Decimal(n), None
        while x != prior: 
            prior = x
            x = (x + n/x) / 2 # quadratic convergence 
    return +x # round in a global context

def signum(x):              #Define Sign Function
    if x > 0:
        return 1
    elif x < 0:
        return -1
    elif x == 0:
        return 0
    else:
        return x

def sinc(x):                #Define Sinc Function
    if x == 0:
        return 1
    if x != 0:
        return math.sin((math.pi*(x)/180)/(math.pi*(x)/180))

# Waypoint
def waypoint():
    x_wp_1 = []
    y_wp_1 = []
    x_wp_2 = []
    y_wp_2 = []
    x_wp_3 = []
    y_wp_3 = []
    x_wp_4 = []
    y_wp_4 = []

    xpos = [2.204,1.312,0.755,0.755,0.755]
    ypos = [-0.251,-0.251,-0.251,-0.909,-1.783]

    # calculate seg 1
    dist_seg_1 = xpos[0]-xpos[1]
    increment_seg_1 = abs(dist_seg_1)/15
    x_wp_1.append(xpos[0])
    y_wp_1.append(ypos[0])
    for i in range(0,15): #segmen 1
        x_wp_1.append(x_wp_1[-1]-increment_seg_1)
        y_wp_1.append(ypos[0])

    # calculate seg 2
    dist_seg_2 = xpos[1]-xpos[2]
    increment_seg_2 = abs(dist_seg_2)/10
    x_wp_2.append(xpos[1])
    y_wp_2.append(ypos[1])
    for i in range(0,10): #segmen 2
        x_wp_2.append(x_wp_2[-1]-increment_seg_2)
        y_wp_2.append(ypos[1])
  
    # calculate seg 3
    dist_seg_3 = ypos[2]-ypos[3]
    increment_seg_3 = abs(dist_seg_3)/10
    x_wp_3.append(xpos[2])
    y_wp_3.append(ypos[2])
    for i in range(0,10): #segmen 3
        y_wp_3.append(y_wp_3[-1]-increment_seg_3)
        x_wp_3.append(xpos[2])
  
    # calculate seg 4
    dist_seg_4 = ypos[3]-ypos[4]
    increment_seg_4 = abs(dist_seg_4)/15
    x_wp_4.append(xpos[3])
    y_wp_4.append(ypos[3])
    for i in range(0,15): #segmen 4
        y_wp_4.append(y_wp_4[-1]-increment_seg_4)
        x_wp_4.append(xpos[3])

    # waypoint gabungan 
    y_wp = y_wp_1 + y_wp_2 + y_wp_3 + y_wp_4
    x_wp = x_wp_1 + x_wp_2 + x_wp_3 + x_wp_4

    # print(x_wp)
    # print(y_wp)
    return xpos, ypos, x_wp, y_wp

# Anti-Jackknife based lyapunov
def anti_jackknife_control(xt_AV, yt_AV, xh_AV, yh_AV, yaw_t, yaw_h, x_wp, y_wp, xpos, ypos):
    # check waypoint to follow based on time trajectory   
    close_idx = np.argmin(np.sum(np.square((xt_AV - x_wp), (yt_AV - y_wp))))
    if((x_wp[close_idx] < xpos[0]) and (y_wp[close_idx] > ypos[3])):
        h_et = yt_AV - y_wp
        h_eh = yh_AV - y_wp
        Qd = 0
        Qh_e = abs((yaw_h - Qd) * math.pi / 180)
        Qt_e = abs((yaw_t - Qd) * math.pi / 180)

    elif(((x_wp[close_idx] > xpos[1]) and (y_wp[close_idx] > ypos[3])) or (x_wp[close_idx] < xpos[3])):
        h_et = xt_AV - x_wp
        h_eh = xh_AV - x_wp    
        Qd = 1.5708
        Qh_e = abs((yaw_h - Qd) * math.pi / 180)
        Qt_e = abs((yaw_t - Qd) * math.pi / 180)
        
       
    # Steering control based lyaponov and velocity control
    if xt_AV == 0 : 
        cs_steer = 0
    else :       
        #Steering Angle Control Function
        Qht_e = Qh_e - Qt_e
        if abs(Qht_e) >= math.pi/5 :
            Qht_e = signum(Qht_e) * math.pi/5
        else :
            Qht_e = Qht_e
        
        vh1 = vh_max / (1 + 0.5*kQt_e * abs(Qt_e) + 0.5*kQh_e * abs(Qht_e) + kh_eh * abs(h_et))
        V_AV = vh1*1000*63/(1100*2)
        psi1 = -math.atan(lt/(vh1)*(0.01*kQt_e*Qt_e/math.cos(Qht_e)+kQt_e*h_et*vh1*sinc(Qt_e/math.pi)))
        z = psi1 - Qht_e
        cs_steer = -math.atan(lh/(vh1)*(-0.01*kQh_e*z+vh1/lt*math.tan(psi1)*math.cos(Qht_e)))

        if abs(cs_steer) >= 19/180 * math.pi :
            cs_steer = signum(cs_steer) * 19 * math.pi / 180
        else :
            cs_steer = cs_steer
        #cs_steer = (dQs * 180 / math.pi)
    
    return V_AV, -cs_steer

# update AV actual state
# subscribe dari vehicle_state
def main_function(msg):

    global xt_AV, yt_AV, xh_AV, yh_AV, yaw_t, yaw_h, V_AV

    xt_AV = msg.xt_est
    yt_AV = msg.yt_est
    xh_AV = msg.xh_est
    yh_AV = msg.yh_est
    yaw_t = msg.yaw_t_est
    yaw_h = msg.yaw_h_est
    V_AV = msg.v_est

# calculate and update actual steer angle
# subscribe dari ESP
def steer_angle_callback(client, userdata, message):
    global cs_steer_actual

    cs_steer_actual = int(message.payload.decode("utf-8"))
    cs_steer_actual = 0.002306 * cs_steer_actual - 1.0898

# MQTT stuff
client = connect_mqtt()
client.subscribe(topic)
client.message_callback_add("/EV3_movement/steer_angle_ADC", steer_angle_callback)
client.loop_start()

# subscribe node vehicle state
sub = rospy.Subscriber('/vehicle_state', State_Estimator, main_function)
# rospy.spin()

# loop utama
while True:
    #======================Controller Part===================================
    cs_long, cs_steer = anti_jackknife_control(xt_AV, yt_AV, xh_AV, yh_AV, yaw_t, yaw_h, V_AV)
    # send command to ev3 via master_auto_control node 
    client.publish("/EV3_movement/steer_command_rad",cs_steer,qos=0)
    client.publish("/EV3_movement/speed_command",cs_long,qos=1)
    # print(dt)  

    # Store calculated value to pub_msg
    # disesuaikan saja
    # AV Actual   
    pub_msg.actual_xt_av = xt_AV
    pub_msg.actual_yt_av = yt_AV
    pub_msg.actual_yaw_t = yaw_t
    pub_msg.actual_speed_av = V_AV
    pub_msg.actual_xh_av = xh_AV
    pub_msg.actual_yh_av = yh_AV
    pub_msg.actual_yaw_h = yaw_h
    pub_msg.actual_steer_av = cs_steer_actual

    pub_msg.error_teta_t = Qt_e
    pub_msg.error_teta_h = Qh_e
    pub_msg.error_teta_ht = Qht_e
    pub_msg.error_t = h_et
    pub_msg.error.h = h_eh
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.seq += 1
    pub.publish(pub_msg)

    rate.sleep()
