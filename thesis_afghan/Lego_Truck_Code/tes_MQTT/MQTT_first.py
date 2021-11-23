import paho.mqtt.client as mqtt
import time

broker ="192.168.0.144"
port = 1883
topic = "/EV3_test"

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

def on_message(client, userdata, message):
    print("received message: " ,str(message.payload.decode("utf-8")))

client = connect_mqtt()
client.subscribe(topic)
client.on_message = on_message
client.loop_forever()