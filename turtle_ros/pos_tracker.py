from geometry_msgs.msg  import TwistStamped
from rclpy.time import Time
import threading
import time


class Pos_tracker:

    def __init__(self, logger, clock):
        self.steps = []
        self.return_stop_flag = threading.Event()
        self.return_thread = None
        self.logger = logger
        self.clock = clock


    def return_to_start(self, publisher, mqtt_transmit):
        self.return_stop_flag.clear()

        while len(self.steps) > 1:
            if self.return_stop_flag.is_set():
                break

            # Debug Print
            self.logger.info(f"Current steps list = {len(self.steps)}.")

            twist_msg = self.steps[-2] # Get twist_msg for the step

            # Check if the robot was stationary in the step
            if twist_msg.twist.linear.x == 0.0 and twist_msg.twist.angular.z == 0.0:
                self.steps.pop()
                continue

            # Calculate the drive time for the step
            drive_time = Time.from_msg(self.steps[-1].header.stamp) - Time.from_msg(twist_msg.header.stamp)
            drive_time = max(drive_time.nanoseconds / 1e9, 0.0)

            # Print statement for debug
            self.logger.info(f"Drive time: {drive_time}, original vel: {twist_msg.twist.linear.x}, {twist_msg.twist.angular.z}.")

            # Calcualte the inverse of the movement done in the step
            twist_msg.twist.linear.x *= -1
            twist_msg.twist.angular.z *= -1

            # Publish twist_msg to cmd_vel and wait for drive_time
            publisher.publish(twist_msg)

            # Send back information to gui
            mqtt_transmit("movement", {"linear": twist_msg.twist.linear.x, "angular": twist_msg.twist.angular.z})

            # Remove instruction from list
            self.steps.pop()

            # Sleep in increments to catch stop flag in short time
            waited = 0.0
            step = 0.05
            while waited < drive_time:
                if self.return_stop_flag.is_set():
                    break
                time.sleep(min(step, drive_time-waited))
                waited += step

        # Stop robot and clear the steps list
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.clock.now().to_msg()
        twist_msg.twist.linear.x = float(0.0)
        twist_msg.twist.angular.z = float(0.0)
        publisher.publish(twist_msg)
        self.steps.clear()

    def save_step(self, twist_msg):
        self.steps.append(twist_msg)
