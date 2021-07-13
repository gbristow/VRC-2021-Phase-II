import paho.mqtt.client as mqtt
from pcc_library import VRC_Peripheral

class PCCModule(object):
    def __init__(self, serial_port):
        self.mqtt_host = "localhost"
        self.mqtt_port = "1883"

        self.mqtt_client = mqtt.Client()

        self.pcc = VRC_Peripheral(serial_port)

        self.topic_prefix = "drone"

        self.topic_map = {
            f"{self.topic_prefix}/pcc/set_base_color": self.set_base_color,
            f"{self.topic_prefix}/pcc/set_temp_color": self.set_temp_color,
            f"{self.topic_prefix}/pcc/set_servo_open_close": self.set_servo_open_close,
            f"{self.topic_prefix}/pcc/set_servo_min": self.set_servo_min,
            f"{self.topic_prefix}/pcc/set_servo_max": self.set_servo_max,
            f"{self.topic_prefix}/pcc/set_servo_pct": self.set_servo_pct,
            f"{self.topic_prefix}/pcc/reset": self.reset,
        }

    def run(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_forever()

    def on_message(self, client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))

        if msg.topic in self.topic_map.keys():
            self.topic_map[msg.topic](msg.payload)

    def on_connect(self, client, userdata, rc):
        print("Connected with result code "+str(rc))
        for topic in self.topic_map.keys():
            print(f"PCCModule: Subscribed to: {topic}")
            client.subscribe(topic)

    def set_base_color(self, payload):
        wrgb = payload["wrgb"]
        self.pcc.set_base_color(wrgb=wrgb)

    def set_temp_color(self, payload):
        wrgb = payload["wrgb"]
        if "time" in payload.keys():
            time = payload["time"]
        else:
            time = 0.5
        self.pcc.set_temp_color(wrgb=wrgb, time=time)

    def set_servo_open_close(self, payload):
        servo = payload["servo"]
        action = payload["action"]
        self.pcc.set_servo_open_close(servo, action)

    def set_servo_min(self, payload):
        servo = payload["servo"]
        pulse = payload["min_pulse"]
        self.pcc.set_servo_min(servo, pulse)

    def set_servo_max(self, payload):
        servo = payload["servo"]
        pulse = payload["max_pulse"]
        self.pcc.set_servo_max(servo, pulse)

    def set_servo_pct(self, payload):
        servo = payload["servo"]
        percent = payload["percent"]
        self.pcc.set_servo_pct(servo, percent)

    def reset(self, payload):
        self.pcc.reset_vrc_peripheral()