import paho.mqtt.client as mqtt
import json
from pcc_library import VRC_Peripheral
from typing import List

class PCCModule(object):
    def __init__(self, serial_port):
        self.mqtt_host = "localhost"
        self.mqtt_port = 1883

        self.mqtt_user = "user"
        self.mqtt_pass = "password"

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(username=self.mqtt_user,password=self.mqtt_pass)

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.pcc = VRC_Peripheral(serial_port, use_serial=False)

        self.topic_prefix = "vrc/pcc"

        self.topic_map = {
            f"{self.topic_prefix}/set_base_color": self.set_base_color,
            f"{self.topic_prefix}/set_temp_color": self.set_temp_color,
            f"{self.topic_prefix}/set_servo_open_close": self.set_servo_open_close,
            f"{self.topic_prefix}/set_servo_min": self.set_servo_min,
            f"{self.topic_prefix}/set_servo_max": self.set_servo_max,
            f"{self.topic_prefix}/set_servo_pct": self.set_servo_pct,
            f"{self.topic_prefix}/reset": self.reset,
        }

    def run(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_forever()

    def on_message(self, client, userdata, msg):
        try:
            print(msg.topic+" "+str(msg.payload))

            if msg.topic in self.topic_map.keys():
                self.topic_map[msg.topic](msg.payload)
        except Exception as e:
            print(f"Error handling message on {msg.topic}")

    def on_connect(self, client, userdata, rc, properties=None):
        print("Connected with result code "+str(rc))
        for topic in self.topic_map.keys():
            print(f"PCCModule: Subscribed to: {topic}")
            client.subscribe(topic)

    def set_base_color(self, payload):
        payload = json.loads(payload)
        wrgb: List = payload["wrgb"]
        self.pcc.set_base_color(wrgb=wrgb)

    def set_temp_color(self, payload):
        payload = json.loads(payload)
        wrgb: List = payload["wrgb"]
        if "time" in payload.keys():
            time: float = payload["time"]
        else:
            time: float = 0.5
        self.pcc.set_temp_color(wrgb=wrgb, time=time)

    def set_servo_open_close(self, payload):
        payload = json.loads(payload)
        servo: int = payload["servo"]
        action: str = payload["action"]
        self.pcc.set_servo_open_close(servo, action)

    def set_servo_min(self, payload):
        payload = json.loads(payload)
        servo: int = payload["servo"]
        pulse: int = payload["min_pulse"]
        self.pcc.set_servo_min(servo, pulse)

    def set_servo_max(self, payload):
        payload = json.loads(payload)
        servo: int = payload["servo"]
        pulse: int = payload["max_pulse"]
        self.pcc.set_servo_max(servo, pulse)

    def set_servo_pct(self, payload):
        payload = json.loads(payload)
        servo: int = payload["servo"]
        percent: int  = payload["percent"]
        self.pcc.set_servo_pct(servo, percent)

    def reset(self, payload):
        payload = json.loads(payload)
        self.pcc.reset_vrc_peripheral()

if __name__ == "__main__":
    pcc = PCCModule("na")
    pcc.run()
