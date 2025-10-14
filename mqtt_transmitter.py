import paho.mqtt.client as mqtt
import json
from time import sleep

# Define MQTT connection details
MQTT_SERVER = "10.52.219.254"   # Replace with MQTT server address
MQTT_PORT = 1883                # Replace with MQTT port
MQTT_TOPIC = "mqtt_vel"         # Replace with MQTT topic

# Callback for when client receives a CONNACK response from server
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connection to MQTT broker successfull")
    else:
        print(f"Failed to connect to MQTT broker")


def publish_command(linear_x, angular_z):
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
    client.publish(MQTT_TOPIC, json.dumps(payload), qos=1)
    print(f"Published to {MQTT_TOPIC}: {payload}")


# Init MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.connect(MQTT_SERVER, MQTT_PORT, 60)
client.loop_start()

sleep(1)

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
            publish_command(linear_x, angular_z)
        except ValueError:
            print("Invalid input. Please enter numerical values for linear.x and angular.z velocities.")

except KeyboardInterrupt:
    print("\nCtrl+C pressed. Sending stop command (linear.x=0, angular.z=0) and disconnecting.")
    # Send stop command with zero velocities
    publish_command(0.0, 0.0)

finally:
    client.loop_stop()
    client.disconnect()
    print("Disconnected from MQTT broker")
