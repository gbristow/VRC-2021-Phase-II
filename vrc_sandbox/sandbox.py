import paho.mqtt.client as mqtt


class Sandbox(object):
    def __init__(self):
        self.mqtt_host = "localhost"
        self.mqtt_port = "1883"

        self.mqtt_client = mqtt.Client()

    def run(self):
        pass

    def on_message(self, client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))

    def on_connect(self, client, userdata, rc):
        print("Connected with result code "+str(rc))
        client.subscribe("SYS/#")