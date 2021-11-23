#!/usr/bin/env python

# Copyright 2017 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from prius_msgs.msg import Control
# edit dikit
# from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from thesis.msg import Autonomous_Game

#for used in master PC (with ROS)
# using MQTT
import paho.mqtt.client as mqtt
import numpy as np

# MQTT Stuff
broker ="192.168.1.101"
port = 1883

STEERING_AXIS = 0
THROTTLE_AXIS = 4

class Translator:
    def __init__(self):
        # edit dikit
        self.sub = rospy.Subscriber("key_vel", Twist, self.callback)
        self.client = Translator.connect_mqtt()
        self.client.loop_start()
    
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

    def callback(self, message):
        rospy.logdebug("joy_translater received axes %s",message)
        if message.linear.x > 0:
            cs_long = 150
        elif message.linear.x < 0:
            cs_long = -150
        else : 
            cs_long = 0

        if message.angular.z > 0:
            cs_steer = 1
        elif message.angular.z < 0:
            cs_steer = -1
        else :
            cs_steer = 0

        self.client.publish("/EV3_movement/steer_command",cs_steer)
        self.client.publish("/EV3_movement/speed_command",cs_long)
        print(cs_steer, cs_long)

if __name__ == '__main__':
    rospy.init_node('joy_translator')
    t = Translator()
    rospy.spin()
