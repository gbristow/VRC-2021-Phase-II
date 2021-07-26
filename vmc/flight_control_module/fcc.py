import json
from typing import Any, Callable, Dict

from loguru import logger
import paho.mqtt.client as mqtt

try:
    from fcc_library import FCC # type: ignore
except ImportError:
    from .fcc_library import FCC

class FCCModule(object):
    def __init__(self):

        self.mqtt_host = "mqtt"
        self.mqtt_port = 18830

        # self.mqtt_user = "user"
        # self.mqtt_pass = "password"

        self.mqtt_client = mqtt.Client()
        # self.mqtt_client.username_pw_set(
        #     username=self.mqtt_user, password=self.mqtt_pass
        # )

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.topic_prefix = "vrc"

        # todo ?
        self.mqtt_topics: Dict[str, Callable[[dict], None]] = {
            f"{self.topic_prefix}/vision/position": self.mocap_queue,
        }

    def run(self) -> None:
        self.mqtt_client.connect(host=self.mqtt_host, port=self.mqtt_port, keepalive=60)
        self.mqtt_client.loop_forever()

    def on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage):
        try:
            logger.debug(f"{msg.topic}: {str(msg.payload)}")
            if msg.topic in self.mqtt_topics.keys():
                data = json.loads(msg.payload)
                self.mqtt_topics[msg.topic].put(data)
        except Exception as e:
            logger.exception(f"Error handling message on {msg.topic}")

    def on_connect(
        self,
        client: mqtt.Client,
        userdata: Any,
        rc: int,
        properties: mqtt.Properties = None,
    ) -> None:
        logger.debug(f"Connected with result code {str(rc)}")
        for topic in self.mqtt_topics.keys():
            logger.debug(f"FCCModule: Subscribed to: {topic}")
            client.subscribe(topic)


if __name__ == "__main__":
    fcc = FCCModule()
    fcc.run()
