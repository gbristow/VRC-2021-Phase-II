import json
import os
import sys
from typing import Tuple

import paho.mqtt.client as mqtt
from PySide6 import QtGui, QtWidgets


class MainWidget(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Bell VRC GUI Control App")
        self.setWindowIcon(
            QtGui.QIcon(os.path.join(os.path.dirname(__file__), "logo.png"))
        )

        self.build()
        self.connect_mqtt()

    def build(self) -> None:
        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)

        # ==========================
        # LEDs
        led_groupbox = QtWidgets.QGroupBox("LEDs")
        led_layout = QtWidgets.QVBoxLayout()
        led_groupbox.setLayout(led_layout)

        red_led_button = QtWidgets.QPushButton("Red")
        red_led_button.setStyleSheet("background-color: red")
        red_led_button.clicked.connect(lambda: self.set_led((255, 255, 0, 0)))
        led_layout.addWidget(red_led_button)

        green_led_button = QtWidgets.QPushButton("Green")
        green_led_button.setStyleSheet("background-color: green")
        green_led_button.clicked.connect(lambda: self.set_led((255, 0, 255, 0)))
        led_layout.addWidget(green_led_button)

        blue_led_button = QtWidgets.QPushButton("Blue")
        blue_led_button.setStyleSheet("background-color: blue")
        blue_led_button.clicked.connect(lambda: self.set_led((255, 0, 0, 255)))
        led_layout.addWidget(blue_led_button)

        clear_led_button = QtWidgets.QPushButton("Clear")
        clear_led_button.setStyleSheet("background-color: white")
        clear_led_button.clicked.connect(lambda: self.set_led((0, 0, 0, 0)))
        led_layout.addWidget(clear_led_button)

        layout.addWidget(led_groupbox, 0, 0, 3, 1)

        # ==========================
        # Servos
        servos_groupbox = QtWidgets.QGroupBox("Servos")
        servos_layout = QtWidgets.QVBoxLayout()
        servos_groupbox.setLayout(servos_layout)

        servo_1_groupbox = QtWidgets.QGroupBox("Servo 1")
        servo_1_layout = QtWidgets.QHBoxLayout()
        servo_1_groupbox.setLayout(servo_1_layout)

        servo_1_open_button = QtWidgets.QPushButton("Open")
        servo_1_open_button.clicked.connect(lambda: self.set_servo(1, "open"))
        servo_1_layout.addWidget(servo_1_open_button)

        servo_1_close_button = QtWidgets.QPushButton("Close")
        servo_1_close_button.clicked.connect(lambda: self.set_servo(1, "close"))
        servo_1_layout.addWidget(servo_1_close_button)

        servos_layout.addWidget(servo_1_groupbox)

        servo_2_groupbox = QtWidgets.QGroupBox("Servo 2")
        servo_2_layout = QtWidgets.QHBoxLayout()
        servo_2_groupbox.setLayout(servo_2_layout)

        servo_2_open_button = QtWidgets.QPushButton("Open")
        servo_2_open_button.clicked.connect(lambda: self.set_servo(2, "open"))
        servo_2_layout.addWidget(servo_2_open_button)

        servo_2_close_button = QtWidgets.QPushButton("Close")
        servo_2_close_button.clicked.connect(lambda: self.set_servo(2, "close"))
        servo_2_layout.addWidget(servo_2_close_button)

        servos_layout.addWidget(servo_2_groupbox)

        servo_3_groupbox = QtWidgets.QGroupBox("Servo 3")
        servo_3_layout = QtWidgets.QHBoxLayout()
        servo_3_groupbox.setLayout(servo_3_layout)

        servo_3_open_button = QtWidgets.QPushButton("Open")
        servo_3_open_button.clicked.connect(lambda: self.set_servo(3, "open"))
        servo_3_layout.addWidget(servo_3_open_button)

        servo_3_close_button = QtWidgets.QPushButton("Close")
        servo_3_close_button.clicked.connect(lambda: self.set_servo(3, "close"))
        servo_3_layout.addWidget(servo_3_close_button)

        servos_layout.addWidget(servo_3_groupbox)

        servo_4_groupbox = QtWidgets.QGroupBox("Servo 4")
        servo_4_layout = QtWidgets.QHBoxLayout()
        servo_4_groupbox.setLayout(servo_4_layout)

        servo_4_open_button = QtWidgets.QPushButton("Open")
        servo_4_open_button.clicked.connect(lambda: self.set_servo(4, "open"))
        servo_4_layout.addWidget(servo_4_open_button)

        servo_4_close_button = QtWidgets.QPushButton("Close")
        servo_4_close_button.clicked.connect(lambda: self.set_servo(4, "close"))
        servo_4_layout.addWidget(servo_4_close_button)

        servos_layout.addWidget(servo_4_groupbox)

        layout.addWidget(servos_groupbox, 0, 1, 3, 3)

        # ==========================
        # Autonomous mode
        autonomous_groupbox = QtWidgets.QGroupBox("Autonomous")
        autonomous_layout = QtWidgets.QHBoxLayout()
        autonomous_groupbox.setLayout(autonomous_layout)

        autonomous_enable_button = QtWidgets.QPushButton("Enable")
        autonomous_enable_button.clicked.connect(lambda: self.set_autonomous(True))
        autonomous_layout.addWidget(autonomous_enable_button)

        autonomous_disable_button = QtWidgets.QPushButton("Disable")
        autonomous_disable_button.clicked.connect(lambda: self.set_autonomous(False))
        autonomous_layout.addWidget(autonomous_disable_button)

        layout.addWidget(autonomous_groupbox, 3, 0, 1, 3)

        # ==========================
        # PCC Reset
        reset_groupbox = QtWidgets.QGroupBox("Reset")
        reset_layout = QtWidgets.QVBoxLayout()
        reset_groupbox.setLayout(reset_layout)

        reset_button = QtWidgets.QPushButton("Reset Peripheals")
        reset_button.setStyleSheet("background-color: yellow")
        reset_button.clicked.connect(lambda: self.publish_message("vrc/pcc/reset", {}))
        reset_layout.addWidget(reset_button)

        layout.addWidget(reset_groupbox, 3, 3, 1, 1)

        self.show()

    def connect_mqtt(self) -> None:
        settings = {}
        settings_file = os.path.join(os.path.dirname(__file__), "settings.json")

        mqtt_host = ""

        # try to load last saved settings from file
        if os.path.exists(settings_file):
            try:
                with open(settings_file, "r") as fp:
                    settings = json.load(fp)
                    mqtt_host = settings["mqtt_host"]
            except:
                # if file is corrupt, delete it
                os.remove(settings_file)

        # ask for input
        mqtt_host = QtWidgets.QInputDialog.getText(
            self, "MQTT Host", "Enter MQTT Host:", text=mqtt_host
        )[0]

        # if no selection made
        if mqtt_host == "":
            sys.exit(0)

        mqtt_port = 18830
        self.mqtt_client = mqtt.Client()

        try:
            # try to connect to MQTT server
            self.mqtt_client.connect(host=mqtt_host, port=mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
        except:
            # display error on failed connection
            QtWidgets.QMessageBox.critical(
                self, "MQTT Error", "Could not connect to MQTT server."
            )
            sys.exit(1)

        # show message for successful connection
        QtWidgets.QMessageBox.information(self, "MQTT Status", "Connected to MQTT server.")

        # save settings
        settings["mqtt_host"] = mqtt_host
        with open(settings_file, "w") as fp:
            json.dump(settings, fp)

    def publish_message(self, topic: str, payload: dict) -> None:
        self.mqtt_client.publish(topic=topic, payload=json.dumps(payload))

    def set_servo(self, number: int, action: str) -> None:
        self.publish_message(
            "vrc/pcc/set_servo_open_close", {"servo": number, "action": action}
        )

    def set_led(self, color: Tuple[int, int, int, int]) -> None:
        self.publish_message("vrc/pcc/set_base_color", {"wrgb": color})

    def set_autonomous(self, state: bool) -> None:
        self.publish_message("vrc/autonomous", {"enable": state})


def main():
    app = QtWidgets.QApplication()

    MainWidget()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
