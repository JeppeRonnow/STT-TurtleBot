import paho.mqtt.client as mqtt
import json

class MQTT_Receiver:
    mqtt_server = ""    # IP for mqtt broker
    mqtt_port = 1883    # Port for mqtt broker
    mqtt_topic = ""     # Topic for mqtt broker
    
    def __init__(self, server, port, topic, DEBUG):
        self.mqtt_server = server   # Set IP of mqtt broker
        self.mqtt_port = port       # Set Port of mqtt broker
        self.mqtt_topic = topic     # Set Topic for mqtt
        
        # MQTT Client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to MQTT broker
        self.mqtt_client.connect(self.mqtt_server, self.mqtt_port, 60)

        # Start the MQTT client loop in a non-blocking way
        self.mqtt_client.loop_start()

        # Subscribe to the MQTT topic
        self.mqtt_client.subscribe(self.mqtt_topic)

        if DEBUG: print(f"Subscribed to MQTT topic: {self.mqtt_topic}")

    # When connecting to the MQTT broker
    def on_connect(self, client, userdata, falgs, rc):
        if DEBUG:
            if rc == 0: print("Connected to MQTT broker succesfully")
            else: print("Failed to connect to MQTT broker, return code %d", rc)
   

    def on_message(self, client, userdata, msg):
        try:
            # Assuming the MQTT message payload is a JSON with 'linear' and 'angular' float
            payload = json.loads(msg.payload.decode())

            # JEPPE ADD CODE HERE TO ACT WHEN MESSAGE COMES IN

        except json.JSONDecodeError:
            if DEBUG: print("Failed to decode JSON from MQTT message.")
        except KeyError as e:
            if DEBUG: print(f"Missing except key in MQTT message: {e}")

