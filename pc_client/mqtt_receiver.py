import paho.mqtt.client as mqtt
import json

class MQTT_Receiver:
    mqtt_server = ""    # IP for mqtt broker
    mqtt_port = 1883    # Port for mqtt broker
    mqtt_topic = "mqtt_status"     # Topic for mqtt broker
    
    def __init__(self, server, DEBUG):
        self.mqtt_server = server   # Set IP of mqtt broker
        self.DEBUG = DEBUG
        
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
        if self.DEBUG:
            if rc == 0: print("Connected to MQTT broker succesfully")
            else: print("Failed to connect to MQTT broker, return code %d", rc)
   

    def on_message(self, client, userdata, msg):
        try:
            # Assuming the MQTT message payload is a JSON with 'linear' and 'angular' float
            payload = json.loads(msg.payload.decode())

            # Handle movement commands
            if payload['type'] == "movement":
                linear = float(payload['linear'])
                angular = float(payload['angular'])

                if self.DEBUG: print(f"[MQTT RECIEVER] Received movement command - Linear: {linear}, Angular: {angular}")
            
            # Handle sensor data
            elif payload['type'] == "sensor":
                if payload['direction'] == "front":
                    sensor = float(payload['value'])
                    if self.DEBUG: print(f"[MQTT RECIEVER] Received front sensor data: {sensor}")
                elif payload['direction'] == "rear":
                    sensor = float(payload['value'])
                    if self.DEBUG: print(f"[MQTT RECIEVER] Received rear sensor data: {sensor}")
                else:
                    if self.DEBUG: print(f"[MQTT RECIEVER] Unknown sensor direction: {payload['direction']}")
            else:
                if self.DEBUG: print(f"[MQTT RECIEVER] Unknown message type: {payload['type']}")

        except json.JSONDecodeError as e:
            if self.DEBUG: print(f"Failed to decode JSON from MQTT message: {e}")

        except KeyError as e:
            if self.DEBUG: print(f"Missing except key in MQTT message: {e}")

        except Exception as e:
            if self.DEBUG: print(f"Error processing MQTT message: {e}")

