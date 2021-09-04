import argparse
import json

import paho.mqtt.client as mqtt

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ip", type=str)
    parser.add_argument("servo", type=int)
    parser.add_argument("action", choices=["open", "close"])
    args = parser.parse_args()

    # mqtt start up
    mqtt_port = 18830
    mqtt_client = mqtt.Client()

    mqtt_client.connect(host=args.ip, port=mqtt_port, keepalive=60)
    mqtt_client.loop_start()

    # publish message
    payload = {"servo": args.servo, "action": args.action}
    mqtt_client.publish(topic=f"vrc/pcc/set_servo_open_close", payload=json.dumps(payload))

    mqtt_client.loop_stop()

if __name__ == "__main__":
    main()