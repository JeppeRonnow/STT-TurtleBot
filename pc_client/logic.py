from typing import List, Optional, Tuple, Union
from mqtt  import Mqtt_Timer

class Logic:
    # move_syn = {"move", "go", "walk", "drive"} Not currently in use
    turn_syn = {"turn", "rotate"}
    stop_syn = {"stop", "halt", "freeze", "wait", "pause"}
    return_syn = {"return", "back"}

    dir_syn = {"left", "right"}
    fwd_syn = {"forward", "straight", "ahead"}
    back_syn = {"backward", "backwards", "reverse"}

    dist_units = {"mm", "millimeter", "millimeters", "cm", "centimeter", "centimeters", "m", "meter", "meters"}

    # Listening delay before text pass
    current_pause = 0


    # Object init
    def __init__(self, move_timer, PAUSE_ITTERATIONS, DEFAULT_TURN_DEG, DEFAULT_DISTANCE, MOVE_VELOCITY, TURN_VELOCITY, DEBUG) -> None:
        self.PAUSE_ITTERATIONS = PAUSE_ITTERATIONS
        self.DEFAULT_TURN_DEG = DEFAULT_TURN_DEG
        self.DEFAULT_DISTANCE = DEFAULT_DISTANCE
        self.MOVE_VELOCITY = MOVE_VELOCITY
        self.TURN_VELOCITY = TURN_VELOCITY
        self.DEBUG = DEBUG

        self.timer = move_timer

        if self.DEBUG: print(f"[Logic class initialized]")


    def handle_transcription(self, words: List[str]) -> Tuple[Optional[str], int]:
        if not words:
            return None, 0

        n = len(words)
        for i, word in enumerate(words):
            # Stop
            if word in self.stop_syn:
                operation = "stop"
                return operation, i + 1
            
            # Return
            if word in self.return_syn:
                operation = "return"
                return operation, i + 1
            
            # Turn left/right
            if word in self.turn_syn:
                operation = "turn"
                direction = None
                distance = None
                last_word_index = i
                # Use the remaining words starting from current position
                for j, current_word in enumerate(words):
                    #if self.DEBUG: print(j, current_word)
                    if current_word in self.dir_syn:
                        direction = current_word
                        if j > last_word_index:
                            last_word_index = j
                    if self.is_number(current_word):
                        distance = self.format_number(current_word)
                        if j > last_word_index:
                            last_word_index = j
                    if direction and distance:
                        break
                if self.await_input(last_word_index, n, distance):
                    return None, 0
                if not distance:
                    distance = self.format_number(self.DEFAULT_TURN_DEG)
                if direction and distance:
                    consumed = last_word_index + 1
                    self.current_pause = 0
                    payload = self.format_payload(operation, direction, distance)
                    return payload, consumed

            # Move
            if word in self.fwd_syn or word in self.back_syn:
                operation = "move"
                direction = "forward" if word in self.fwd_syn else "backward"
                distance = None
                unit = None
                last_word_index = i
                for j, current_word in enumerate(words):
                    #if self.DEBUG: print(j, current_word)
                    if self.is_number(current_word):
                        distance = self.format_number(current_word)
                        if j > last_word_index: 
                            last_word_index = j
                    if current_word in self.dist_units:
                        unit = current_word
                        if j > last_word_index:
                            last_word_index = j
                    if distance and unit:
                        break
                if self.await_input(last_word_index, n, distance, unit):
                    return None, 0
                if not distance:
                    distance = self.format_number(self.DEFAULT_DISTANCE)
                if unit:
                    distance = self.format_unit(distance, unit)
                if distance:
                    consumed = last_word_index + 1
                    self.current_pause = 0
                    payload = self.format_payload(operation ,direction, distance)
                    return payload, consumed

        return None, 0


    @staticmethod
    def is_number(number: str) -> bool:
        try:
            float(number)
            return True
        except ValueError:
            return False


    @staticmethod
    def format_unit(value: float, unit: Optional[str]) -> float:
        if not unit:
            return float(value)
        if unit in ("cm", "centimeter", "centimeters"):
            return float(value / 100)
        if unit in ("mm", "millimeter", "millimeters"):
            return float(value / 1000)
        return float(value)


    @staticmethod
    def format_number(number) -> float:
        return float(number)


    @staticmethod
    def format_payload(operation: str, *args: Union[str, float]) -> str:
        if not args:
            return operation
        if len(args) == 2 and isinstance(args[0], str) and isinstance(args[1], float):
            if args[0] in ["left", "backward"]:
                return f"{operation} -{args[1]}"
            else: 
                return f"{operation} {args[1]}"
        raise TypeError("Invalid format_payload arguments")
    

    def await_input(self, last_word_index, n, distance, unit=True) -> bool:
        if last_word_index >= n and self.current_pause < self.PAUSE_ITTERATIONS:
            if self.DEBUG: print("LAST WORD")
            if self.DEBUG: print(f"Pausing for more input... ({self.current_pause}/{self.PAUSE_ITTERATIONS})")
            self.current_pause += 1
            return True

        elif not distance and self.current_pause < self.PAUSE_ITTERATIONS:
            if self.DEBUG: print("NOT DISTANCE")
            if self.DEBUG: print(f"Pausing for more input... ({self.current_pause}/{self.PAUSE_ITTERATIONS})")
            self.current_pause += 1
            return True

        elif not unit and self.current_pause < self.PAUSE_ITTERATIONS:
            if self.DEBUG: print("NOT UNIT")
            if self.DEBUG: print(f"Pausing for more input... ({self.current_pause}/{self.PAUSE_ITTERATIONS})")
            self.current_pause += 1
            return True

        return False


    # Convert payload to velocities
    def payload_to_velocities(self, payload) -> Tuple[float, float]:
        payload = payload.split(" ")

        # Stop any existing timers
        self.timer.stop_timer()

        if payload[0] == "stop":
            return (0.0, 0.0)

        if payload[0] == "return":
            return (69.69, 69.69)

        if payload[0] == "turn" and payload[1]:
            val = float(payload[1])

            if val == 69.69 or val == -69.69:
                return (0.0, self.TURN_VELOCITY) # Turn right
            else:
                self.timer.set_timer(payload[0], val)
                if val > 0:
                    return (0.0, -self.TURN_VELOCITY)
            return (0.0, self.TURN_VELOCITY)

        if payload[0] == "move" and payload[1]:
            val = float(payload[1])
            
            if val == 69.69 or val == -69.69:
                return (self.MOVE_VELOCITY, 0.0) # Move forward 
            
            else:
                self.timer.set_timer(payload[0], val)
                if val > 0:
                    return (self.MOVE_VELOCITY, 0.0)
                return (-self.MOVE_VELOCITY, 0.0)
        
        return (0.0, 0.0)
    

if __name__ == "__main__":
    from mqtt import MQTT_Transmitter
    from config import Config
    from logic import Logic
    from stt import STT
    import time

    # Configuration parameters
    config = Config() # Load config variables from YAML file
    mqtt = MQTT_Transmitter(config.SERVER, config.DEBUG)
    logic = Logic(mqtt ,config.PAUSE_ITTERATIONS, config.DEFAULT_TURN_DEG, config.DEFAULT_DISTANCE, config.MOVE_VELOCITY, config.TURN_VELOCITY, config.DEBUG)                                # Logic control
    whisper = STT(config.MODEL_NAME, config.MODEL_DEVICE, config.BUFFER_SECONDS, config.SAMPLE_RATE, config.MAX_BUFFER_LENGTH, config.DEBUG)  # Speach to Text

    # Load Whisper STT model
    model = whisper.load_model()

    string = "Stop"
    for word in string.split():
        whisper.add_transcription(word)
        words = whisper.get_transcription()
    while words:
        whisper.print_transcription()
        words = whisper.get_transcription()
        payload, consumed = logic.handle_transcription(words)
        if payload:
            print("[Payload]:", payload)
            velocities = logic.payload_to_velocities(payload)
            print("[Velocities]:", velocities)
            mqtt.publish_command(velocities[0], velocities[1])
        if consumed:
            print("[Consumed]:", consumed)
            whisper.strip_transcription(consumed)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        mqtt.publish_command(0.0, 0.0)
