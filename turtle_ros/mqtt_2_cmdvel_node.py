import json
import threading

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import TwistStamped
from mqtt_2_cmd_pkg.pos_tracker import Pos_tracker
from mqtt_2_cmd_pkg.tof_sensor import ToFSensor
from rclpy.node import Node

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

# Constrain a velcoty within upper and lower bound
def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


# Constrain Linear velocity
def check_linear_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)


# Constrain angular velocity
def check_angular_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)


class MqttToCmdVelNode(Node):
    def __init__(self):
        super().__init__("mqtt_to_cmd_vel")

        # Create a ROS2 publisher to publish on the cmd_vel topic
        self.publisher = self.create_publisher(TwistStamped, "cmd_vel", 10)

        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Define MQTT connection details
        mqtt_server = "127.0.0.1"
        mqtt_port = 1883
        mqtt_topic = "mqtt_vel"

        # Connect to the MQTT broker
        self.mqtt_client.connect(mqtt_server, mqtt_port, 60)

        # Start the MQTT client loop in a non-blocking way
        self.mqtt_client.loop_start()

        # Subscribe to the MQTT topic
        self.mqtt_client.subscribe(mqtt_topic)
        self.get_logger().info(f"Subscribed to MQTT topic: {mqtt_topic}")
        
        # Init Position tracking for robot to enable return for the robot
        self.Pos_tracker = Pos_tracker(self.get_logger(), self.get_clock()) # Tracks position of robot

        # Init obstacle detection class
        self.tof = ToFSensor(self.get_logger())


    # When connecting to MQTT broker
    def on_connect(self, client, userdata, falgs, rc):
        if rc == 0:
            self.get_logger().info("Connect to MQTT broker succesfully")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, return code: {rc}")


    # When message is received
    def on_message(self, client, userdata, msg):
        try:
            # Assuming the MQTT message payload is a JSON with 'linear' and 'angualr' fields
            payload = json.loads(msg.payload.decode())

            # Check if comand is return
            if (payload['linear']['x'] == 69.69 and payload['angular']['z'] == 69.69):
                self.start_return()
                return

            # Check if return is active
            self.stop_return_thread_if_active()

            # Check if collision detection is active before we start a new thread
            self.stop_collision_thread_if_active()

            # Check if meassage fit within constraints, and constrain if nescesarry
            linear_vel = check_linear_limit_velocity(payload['linear']['x'])
            angular_vel = check_angular_limit_velocity(payload['angular']['z'])

            # Create a Twist message
            twist_msg = self.create_twist_msg(linear_vel, angular_vel)

            # Publish to cmd_vel topic
            self.publisher.publish(twist_msg)
            self.get_logger().info(
                f"Published cmd_vel: linear={twist_msg.twist.linear.x}, angualr={twist_msg.twist.angular.z}"
            )

            # Start collision_thread
            self.start_collision_detection(linear_vel)

            # Save Position.
            self.Pos_tracker.save_step(twist_msg) # Save velocities and time to list of steps

        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON from MQTT message.")
        except KeyError as e:
            self.get_logger().error(f"Missing except key in MQTT message: {e}")


    # Create twist message
    def create_twist_msg(self, linear_vel, angular_vel):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = float(linear_vel)
        twist_msg.twist.angular.z = float(angular_vel)
        return twist_msg


    # Starts the return of the robot
    def start_return(self):
        # Make sure robot is stationary and nescesarry for time calculation if return is called while robot is moveing
        twist_msg_stop = self.create_twist_msg(0.0, 0.0)
        self.Pos_tracker.save_step(twist_msg_stop)
        
        # If return method is already running continue returning and wait for new msg
        if self.Pos_tracker.return_thread and self.Pos_tracker.return_thread.is_alive():
            return

        # Setup thread to run the return method and start it
        self.Pos_tracker.return_thread = threading.Thread(
            target=self.Pos_tracker.return_to_start,
            args=(self.publisher,),
            daemon=True
        )
        self.Pos_tracker.return_thread.start()


    # Start thread for collision detection 
    def start_collision_detection(self, linear_vel):
        self.tof.collision_thread = threading.Thread(
            target=self.collision_detection,
            args=(linear_vel),
            daemon=True
        )
        self.tof.collision_thread.start()


    # Checks if the return_thread is active and stops it if it active
    def stop_return_thread_if_active(self):
        if self.Pos_tracker.return_thread and self.Pos_tracker.return_thread.is_alive():
                self.Pos_tracker.return_stop_flag.set()
                self.Pos_tracker.return_thread.join()


    # Checks if the collision_thread is active and stops it if it active
    def stop_collision_thread_if_active(self):
        if self.tof.collision_thread and self.tof.collision_thread.is_alive():
                self.tof.collision_thread_flag.set()    # Set stop flag for collision_thread
                self.tof.collision_thread.join()        # Wait for collision_thread to stop safely


    # Is run as a thread from the on_message function
    def collision_detection(self, linear_vel):
        if linear_vel > 0.0:
            direction = "front"
        else:
            direction = "rear"

        collision = False

        # Read sensor for collision
        while not self.tof.collision_thread_flag.is_set():
            # Keep checking if there is a collision
            collision = self.tof.stream(direction)

            if collision:
                break

        # Publish stop velocities only in case of collision
        if collision:
            twist_msg = self.create_twist_msg(0.0, 0.0)
            self.publisher.publish(twist_msg)

            # Check if the return thread is active and if active stop it
            self.stop_return_thread_if_active()

            self.Pos_tracker.save_step(twist_msg) # Save the stop steps


def main(args=None):
    rclpy.init(args=args)
    node = MqttToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
