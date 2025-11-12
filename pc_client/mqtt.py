import paho.mqtt.client as mqtt
from time import sleep
import threading
import json
import math


class MQTT_Transmitter:
    # Define MQTT connection details
    MQTT_SERVER = ""   # Replace with MQTT server address
    MQTT_PORT = 1883      # Replace with MQTT port
    MQTT_TOPIC = "mqtt_vel"    # Replace with MQTT topic


    # Class init and mqtt init
    def __init__(self, server, DEBUG) -> None:
        self.MQTT_SERVER = server
        self.DEBUG = DEBUG

        if self.DEBUG: print(f"\nConnecting to MQTT broker at {self.MQTT_SERVER}:{self.MQTT_PORT}...")

        # Init MQTT client
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.connect(self.MQTT_SERVER, self.MQTT_PORT, 60)
        self.client.loop_start()
        if self.DEBUG: print(f"[MQTT class initialized]")

        sleep(1)


    # Callback for when client receives a CONNACK response from server
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            if self.DEBUG: print("Connection to MQTT broker successfull")
        else:
            if self.DEBUG: print(f"Failed to connect to MQTT broker")


    # Closes connection to mqtt correct
    def close_connectio(self):
        self.publish_command(0.0, 0.0) # Make sure robot stops 
        self.client.loop_stop()
        self.client.disconnect()
        if self.DEBUG: print("Disconnected from MQTT broker")


    # Sends linear_x and angular_z velocities
    def publish_command(self, linear_x, angular_z):
        # Prepare payload to match a Twist message structure
        payload = {
            "linear": {
                "x": linear_x,
                "y": 0.0,
                "z": 0.0
            },
            "angular": {
                "x": 0.0,
                "y": 0.0,
                "z": angular_z
            }
        }

        # Publish payload to the MQTT topic
        self.client.publish(self.MQTT_TOPIC, json.dumps(payload), qos=1)
        if self.DEBUG: print(f"Published to {self.MQTT_TOPIC}: {payload}")


# Test code if class is run as main
if __name__ == '__main__':
    # Get mqtt host information
    server = "10.17.116.254"
    port = 1883
    topic = "mqtt_vel"

    # Init mqtt client
    mqtt = MQTT_Transmitter(server, DEBUG=True)

    try:
        print("Enter 'linear.x' and 'angular.z' values to send commands (or 'q' to quit):")

        while True:
            # Get user input
            linear_input = input("Enter linear_x velocity: ")
            if linear_input.lower() == 'q':
                break
            angular_input = input("Enter angular_z velocity: ")
            if angular_input.lower() == 'q':
                break

            # Convert input to floats
            try:
                linear_x = float(linear_input)
                angular_z = float(angular_input)
                mqtt.publish_command(linear_x, angular_z)
            except ValueError:
                print("Invalid input. Please enter numerical values for linear.x and angular.z velocities.")

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Sending stop command (linear.x=0, angular.z=0) and disconnecting.")

        # Send stop command with zero velocities
        mqtt.publish_command(0.0, 0.0)

    finally:
        mqtt.close_connectio()


class Mqtt_Timer:
    def __init__(self, mqtt_class, MOVE_VELOCITY, TURN_VELOCITY, DEBUG: bool = False):
        self.mqtt = mqtt_class
        self.MOVE_VELOCITY = MOVE_VELOCITY
        self.TURN_VELOCITY = TURN_VELOCITY
        self.DEBUG = DEBUG
        self.move_timer = None


    def on_timeout(self) -> None:
        if self.DEBUG: print(f"[Move Timer] Time elapsed, Stopping...")
        self.mqtt.publish_command(0.0, 0.0)


    def calculate_delay(self, operation: str, distance: float) -> float:
        if operation == "move": 
            return abs(distance) / self.MOVE_VELOCITY
        if operation == "turn":
            return math.radians(abs(distance)) / self.TURN_VELOCITY
        raise ValueError("Invalid mode for delay calculation")        


    def stop_timer(self) -> None:
        if self.move_timer and self.move_timer.is_alive():
            if self.DEBUG: print(f"[Move Timer] Cancelling previous timer.")
            self.move_timer.cancel()


    def set_timer(self, operation: str, distance: float) -> None:
        delay = self.calculate_delay(operation, distance)

        if self.DEBUG: print(f"[Move Timer] Setting timer for {delay:.2f} seconds.")
        self.move_timer = threading.Timer(delay, self.on_timeout)
        self.move_timer.daemon = True
        self.move_timer.start()