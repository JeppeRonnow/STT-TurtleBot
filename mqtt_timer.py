import threading
import math

class MqttTimer:
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