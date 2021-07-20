import paho.mqtt.client as mqtt
import time


class Sandbox(object):
    def __init__(self):
        self.mqtt_host = "localhost"
        self.mqtt_port = 1883

        self.mqtt_user = "user"
        self.mqtt_pass = "password"

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.topic_prefix = "vrc"

        self.topic_map = {
            f"{self.topic_prefix}/velocity": self.set_base_color,
        }

    def run(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_forever()

        while True:
            time.sleep(.1)

    def on_message(self, client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))

    def on_connect(self, client, userdata, rc):
        print("Connected with result code "+str(rc))
        client.subscribe("SYS/#")

    #write your custom message handlers here