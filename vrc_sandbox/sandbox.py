# This imports the "time" module from the Python standard library
# https://docs.python.org/3/library/time.html
import time

# This is outside the scope of beginner Python and VRC, but this is for
# something called "type-hinting" that makes Python code easier to debug
from typing import Any, Callable, Dict

# This imports the Paho MQTT library that we will use to communicated to
# other running processes
# https://github.com/eclipse/paho.mqtt.python
import paho.mqtt.client as mqtt

# This creates a new class that will contain multiple functions called "methods"
class Sandbox(object):
    # The "__init__" method of any class is special in Python. It's what runs when
    # you create a class like `sandbox = Sandbox()`. In here, we usually put
    # first-time initialization and setup code. The "self" argument is a magic
    # argument that must be the first argument in any class method. This allows the code
    # inside the method to access class information.
    def __init__(self) -> None:
        # Create a string attribute to hold the hostname/ip address of the MQTT server
        # we're going to connect to (an attribute is just a variable in a class).
        # In this case, the MQTT server will be running
        # on the same computer as this code, so we can use the special name
        # "localhost". https://en.wikipedia.org/wiki/Localhost
        self.mqtt_host = "localhost"
        # Create an integer attribute to hold the port number of the MQTT server
        # we're going to connect to. MQTT uses a default of port 1883, which we will
        # also use.
        self.mqtt_port = 1883
        # Create an attribute to hold an instance of the Paho MQTT client class
        self.mqtt_client = mqtt.Client()

        # This part is a little bit more complicated. Here, we're assigning the
        # attributes of the Paho MQTT client `on_connect` and `on_message` to handles
        # of methods in our Sandbox class, which are defined below.
        # This isn't *running* those methods, but rather creating a reference to them.
        # Once we start running the Paho MQTT client, this tells the client to execute
        # these methods after it establishes the connection, and after every message
        # it recieves, respectfully.
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Create a string attribute to hold the commonly used prefix used in MQTT topics
        self.topic_prefix = "vrc"

        # Here, we're creating a dictionary of MQTT topic names to method handles
        # (as we discussed above). A dictionary is a data structure that allows use to
        # obtain values based on keys. Think of a dictionary of state names as keys
        # and their capitals as values. By using thestate name as a key, you can easily
        # find the associated capital. However, this does not work in reverse. So here,
        # we're creating a dictionary of MQTT topics, and the methods we want to run
        # whenever a message arrives on that topic.
        self.topic_map: Dict[str, Callable] = {
            # This is what is known as a "f-string". This allows you to easily inject
            # variables into a string without needing to add lots of string together.
            # https://realpython.com/python-f-strings/#f-strings-a-new-and-improved-way-to-format-strings-in-python
            f"{self.topic_prefix}/velocity": self.show_velocity,
        }

    # Create a new method to effectively run everything.
    def run(self) -> None:
        # Connect the Paho MQTT client to the MQTT server with the given host and port
        # The 60 is a keep-alive timeout that defines how long in seconds
        # the connection should stay alive if connection is lost.
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
        # This method of the Paho MQTT client tells it to start running in a loop
        # forever until it is stopped.
        self.mqtt_client.loop_forever()

        # As the above function returns immediately, we need to loop forever,
        # otherwise this function would exit immediately. A simple way to do this is
        # create a while True loop. However, adding a short sleep inside the loop
        # is important, as otherwise the loop would run as fast a possible and use
        # all the CPU of the computer. Artifically limiting this to 10 times per second
        # makes this effectively use no CPU power. This is why we imported the time
        # library at the top of the file.
        while True:
            time.sleep(0.1)

    # As we desribed above, this method runs after the Paho MQTT client has connected
    # to the server. This is generally used to do any setup work after the connection
    # and subscribe to topics.
    def on_connect(self, client: mqtt.Client, userdata: Any, rc: int) -> None:
        # Print the result code to the console for debugging purposes.
        print(f"Connected with result code {str(rc)}")
        # After the MQTT client has connected to the server, this line has the client
        # connect to all topics that begin with our common prefix. The "#" character
        # acts as a wildcard. If you only wanted to subscribe to certain topics,
        # you would run this function multiple times with the exact topics you wanted
        # each time, such as:
        # client.subscribe(f"{self.topic_prefix}/velocity")
        # client.subscribe(f"{self.topic_prefix}/location")
        client.subscribe(f"{self.topic_prefix}/#")

    # As we desribed above, this method runs after any message on a topic
    # that has been subscribed to has been recieved.
    def on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        # Print the topic name and the message payload to the console
        # for debugging purposes.
        print(f"{msg.topic}: f{str(msg.payload)}")

        # First, check if the topic of the message we've recieved is inside the topic
        # map we've created.
        if msg.topic in self.topic_map:
            # If so, lookup the method for the topic, and execute it
            # (with the parentheses) and pass it the payload of the message.
            self.topic_map[msg.topic](msg.payload)

    # ================================================================================
    # Now the training wheels come off! Write your custom message handlers here.
    # Below is a very simple example to look at it.
    def show_velocity(self, data: dict) -> None:
        vx = data["vX"]
        vy = data["vY"]
        vz = data["vZ"]
        v_ms = (vx, vy, vz)
        print(f"Velocity information: {v_ms} m/s")

        # v_fts = tuple([v * 3.28084 for v in v_ms])
        # print(f"Velocity information: {v_fts} ft/s")