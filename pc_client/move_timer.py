import threading
import math


class Move_Timer:
    def __init__(self, mqtt_class, MOVE_VELOCITY, TURN_VELOCITY, DEBUG: bool = False):
        self.mqtt = mqtt_class
        self.timer_thread = None
        self.MOVE_VELOCITY = MOVE_VELOCITY
        self.TURN_VELOCITY = TURN_VELOCITY
        self.DEBUG = DEBUG


    def on_timeout(self) -> None:
        if self.DEBUG: print(f"[Move Timer] Time elapsed, Stopping...")
        self.mqtt.publish_command(0.0, 0.0)


    def calculate_delay(self, operation: str, distance: float) -> float:
        if operation == "move": 
            return abs(distance) / self.MOVE_VELOCITY
        if operation == "turn":
            return math.radians(abs(distance) * 1.04) / self.TURN_VELOCITY
        raise ValueError("Invalid mode for delay calculation")        


    def stop_timer(self) -> None:
        if self.timer_thread and self.timer_thread.is_alive():
            if self.DEBUG: print(f"[Move Timer] Cancelling previous timer.")
            self.timer_thread.cancel()


    def set_timer(self, operation: str, distance: float) -> None:
        delay = self.calculate_delay(operation, distance)

        if self.DEBUG: print(f"[Move Timer] Setting timer for {delay:.2f} seconds.")
        self.timer_thread = threading.Timer(delay, self.on_timeout)
        self.timer_thread.daemon = True
        self.timer_thread.start()
