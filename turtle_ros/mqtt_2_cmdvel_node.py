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

        self.Pos_tracker = Pos_tracker(self.get_logger(), self.get_clock())

        # --- ToF sensor logic ---
        self.tof_threshold = 200  # stop threshold (mm)
        self.tof_poll_interval = 0.1  # sampling frequency
        # Flags for obstacles
        self.front_obstacle = False
        self.rear_obstacle = False

        # Init ToF wrapper (handles XSHUT pins etc.)
        try:
            # xshut pins
            self.tof = ToFSensor(xshut_front=18, xshut_rear=23)
            self.tof.start_stream(
                "front",
                interval=self.tof_poll_interval,
                range_mode=1,
                callback=self._tof_callback,
            )

            self.current_stream = "front"
            self.get_logger().info("ToF front stream started.")
        except Exception as e:
            self.get_logger().warn(f"ToF initialization failed: {e} — ToF disabled.")
            self.tof = None

    #
    def _tof_callback(self, which, distance):
        """
        Callback from ToFSensor.start_stream. 'which' is 'front' or 'rear'.
        'distance' is int (mm) or None.
        Setter: front_obstacle/rear_obstacle and publishes STOP if detektion.
        """
        try:
            if distance is None:
                return
            self.get_logger().debug(f"ToF {which}: {distance} mm")

            if which == "front":
                if distance <= self.tof_threshold:
                    if not self.front_obstacle:
                        self.front_obstacle = True
                        self.get_logger().info(
                            f"Front obstacle at {distance:.3f} mm — publishing STOP"
                        )
                        stop_msg = TwistStamped()
                        stop_msg.header.stamp = self.get_clock().now().to_msg()
                        stop_msg.twist.linear.x = 0.0
                        stop_msg.twist.angular.z = 0.0
                        self.publisher.publish(stop_msg)
                else:
                    if self.front_obstacle:
                        self.front_obstacle = False
                        self.get_logger().info(
                            "Front obstacle cleared — allowing forward commands."
                        )
            elif which == "rear":
                if distance <= self.tof_threshold:
                    if not self.rear_obstacle:
                        self.rear_obstacle = True
                        self.get_logger().info(
                            f"Rear obstacle at {distance:.3f} mm — publishing STOP"
                        )
                        stop_msg = TwistStamped()
                        stop_msg.header.stamp = self.get_clock().now().to_msg()
                        stop_msg.twist.linear.x = 0.0
                        stop_msg.twist.angular.z = 0.0
                        self.publisher.publish(stop_msg)
                else:
                    if self.rear_obstacle:
                        self.rear_obstacle = False
                        self.get_logger().info(
                            "Rear obstacle cleared — allowing reverse commands."
                        )
        except Exception as e:
            self.get_logger().error(f"Error in ToF callback: {e}")

    # When connecting to MQTT broker
    def on_connect(self, client, userdata, falgs, rc):
        if rc == 0:
            self.get_logger().info("Connect to MQTT broker succesfully")
        else:
            self.get_logger().error(
                "Failed to connect to MQTT broker, return code: %d", rc
            )

    # When message is received
    def on_message(self, client, userdata, msg):
        try:
            # Assuming the MQTT message payload is a JSON with 'linear' and 'angualr' fields
            payload = json.loads(msg.payload.decode())

            # Check if comand is return
            if payload["linear"]["x"] == 69.69 and payload["angular"]["z"] == 69.69:
                self.Pos_tracker.save_step(self.create_twist_msg(0.0, 0.0))

                if (
                    self.Pos_tracker.return_thread
                    and self.Pos_tracker.return_thread.is_alive()
                ):
                    return

                self.Pos_tracker.return_thread = threading.Thread(
                    target=self.Pos_tracker.return_to_start,
                    args=(self.publisher,),
                    daemon=True,
                )
                self.Pos_tracker.return_thread.start()
                return

            # Get requested linear and angular commands
            linear_cmd = payload.get("linear", {}).get("x", 0.0)
            angular_cmd = payload.get("angular", {}).get("z", 0.0)

            # If Pos_tracker return thread is running, signal it to stop
            if (
                self.Pos_tracker.return_thread
                and self.Pos_tracker.return_thread.is_alive()
            ):
                self.Pos_tracker.return_stop_flag.set()

            # Block movement based on ToF flags (flags are set in _tof_callback using mm)
            # Block forward if front_obstacle True
            if linear_cmd > 0.0 and self.front_obstacle:
                self.get_logger().warn(
                    "Ignoring forward cmd_vel due to front ToF obstacle."
                )
                stop_msg = TwistStamped()
                stop_msg.header.stamp = self.get_clock().now().to_msg()
                stop_msg.twist.linear.x = 0.0
                stop_msg.twist.angular.z = 0.0
                self.publisher.publish(stop_msg)
                return

            # Block reverse if rear_obstacle True
            if linear_cmd < 0.0 and self.rear_obstacle:
                self.get_logger().warn(
                    "Ignoring reverse cmd_vel due to rear ToF obstacle."
                )
                stop_msg = TwistStamped()
                stop_msg.header.stamp = self.get_clock().now().to_msg()
                stop_msg.twist.linear.x = 0.0
                stop_msg.twist.angular.z = 0.0
                self.publisher.publish(stop_msg)
                return

            # Only switch ToF sensor if necessary (avoid frequent switches)
            if self.tof is not None:
                desired_sensor = "front" if linear_cmd > 0 else "rear"
                if getattr(self, "current_stream", None) != desired_sensor:
                    try:
                        self.tof.switch_sensor(desired_sensor)
                        self.current_stream = desired_sensor
                    except Exception as e:
                        self.get_logger().warn(f"ToF switch_sensor failed: {e}")

            # Enforce velocity limits and publish
            linear_vel = check_linear_limit_velocity(linear_cmd)
            angular_vel = check_angular_limit_velocity(angular_cmd)

            twist_msg = self.create_twist_msg(linear_vel, angular_vel)

            # Publish to cmd_vel topic
            self.publisher.publish(twist_msg)
            self.get_logger().info(
                f"Published cmd_vel: linear={twist_msg.twist.linear.x}, angualr={twist_msg.twist.angular.z}"
            )

            self.Pos_tracker.save_step(twist_msg)

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


def main(args=None):
    rclpy.init(args=args)
    node = MqttToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()

        # cleanup ToF hardware if available
        try:
            if hasattr(node, "tof") and node.tof is not None:
                node.tof.cleanup()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
