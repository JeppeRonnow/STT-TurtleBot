import paho.mqtt.client as mqtt
import json
from time import sleep

class MQTT_Transmitter:
    # Define MQTT connection details
    MQTT_SERVER = ""   # Replace with MQTT server address
    MQTT_PORT = 1883      # Replace with MQTT port
    MQTT_TOPIC = "mqtt_vel"    # Replace with MQTT topic

    # Class init and mqtt init
    def __init__(self, server) -> None:
        self.MQTT_SERVER = server

        # Init MQTT client
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.connect(self.MQTT_SERVER, self.MQTT_PORT, 60)
        self.client.loop_start()

        sleep(1)


    # Callback for when client receives a CONNACK response from server
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connection to MQTT broker successfull")
        else:
            print(f"Failed to connect to MQTT broker")


    # Closes connection to mqtt correct
    def close_connectio(self):
        self.publish_command(0.0, 0.0) # Make sure robot stops 
        self.client.loop_stop()
        self.client.disconnect()
        print("Disconnected from MQTT broker")


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
        print(f"Published to {self.MQTT_TOPIC}: {payload}")


# Test code if class is run as main
if __name__ == '__main__':
    # Get mqtt host information
    server = "10.132.133.254"
    port = 1883
    topic = "mqtt_vel"

    # Init mqtt client
    mqtt = mqtt_transmitter(server, port, topic)

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
