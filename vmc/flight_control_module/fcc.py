import paho.mqtt.client as mqtt
import json
import queue
from fcc_library import FCC


class FCCModule(object):
    def __init__(self):

        self.mqtt_host = "localhost"
        self.mqtt_port = 1883

        self.mqtt_user = "user"
        self.mqtt_pass = "password"

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(
            username=self.mqtt_user, password=self.mqtt_pass
        )
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.topic_prefix = "vrc"

        self.mqtt_topics = {
            f"{self.topic_prefix}/vision/position": self.mocap_queue,
        }

    def on_message(self, client, userdata, msg):
        try:
            print(msg.topic + " " + str(msg.payload))
            if msg.topic in self.mqtt_topics.keys():
                data = json.loads(msg.payload)
                self.mqtt_topics[msg.topic].put(data)
        except Exception as e:
            print(f"Error handling message on {msg.topic}")

    def on_connect(self, client, userdata, rc, properties=None):
        print("Connected with result code " + str(rc))
        for topic in self.mqtt_topics.keys():
            print(f"FCCModule: Subscribed to: {topic}")
            client.subscribe(topic)


if __name__ == "__main__":
    fcc = FCCModule()

    fcc.run()
