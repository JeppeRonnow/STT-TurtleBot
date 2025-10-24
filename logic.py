from typing import List, Optional, Tuple, overload, Union

from config import PAUSE_ITTERATIONS, DEFAULT_TURN_DEG, PAUSE_ITTERATIONS, DEFAULT_DISTANCE_CM

class Logic:
    move_syn = {"move", "go", "walk", "drive"}
    turn_syn = {"turn", "rotate"}
    stop_syn = {"stop", "halt", "freeze", "wait", "pause"}
    return_syn = {"return", "back"}

    dir_syn = {"left", "right"}
    fwd_syn = {"forward", "straight", "ahead"}
    back_syn = {"backward", "backwards", "reverse"}

    deg_units = {"deg", "degree", "degrees"}
    dist_units = {"mm", "millimeter", "millimeters", "cm", "centimeter", "centimeters", "m", "meter", "meters"}

    # Listening delay before text pass
    current_pause = 0


    # Object init
    def __init__(self) -> None:
        pass


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
                last_word_index = 0
                # Use the remaining words starting from current position
                for j in range(i, n):
                    current_word = words[j]
                    if current_word in self.dir_syn:
                        direction = current_word
                        if j + 1 > last_word_index:
                            last_word_index = j + 1
                    if self.is_number(current_word):
                        distance = self.format_number(current_word)
                        if j + 1 > last_word_index: 
                            last_word_index = j + 1
                    if direction and distance:
                        break
                if last_word_index >= n and not distance and self.current_pause < PAUSE_ITTERATIONS:
                    continue
                if not distance:
                    distance = DEFAULT_TURN_DEG
                if direction and distance:
                    consumed = last_word_index + 1
                    payload = self.format_payload(operation, direction, distance)
                    return payload, consumed

            # Move
            if word in self.fwd_syn or word in self.back_syn:
                operation = "move"
                direction = "forward" if word in self.fwd_syn else "backward"
                distance = None
                unit = None
                last_word_index = 0
                for i, word in enumerate(words):
                    if self.is_number(word):
                        distance = self.format_number(word)
                        if i + 1 > last_word_index: 
                            last_word_index = i + 1
                    if word in self.dist_units:
                        unit = word
                        if i + 1 > last_word_index: 
                            last_word_index = i + 1
                    if distance and unit:
                        break
                if last_word_index >= n and not distance and not unit and self.current_pause < PAUSE_ITTERATIONS:
                    continue
                if not distance:
                    distance = DEFAULT_DISTANCE_CM
                if unit:
                    distance = self.format_unit(distance, unit)
                if distance:
                    consumed = last_word_index + 1
                    payload = self.format_payload(operation ,direction, distance)
                    return payload, consumed

        self.current_pause += 1
        return None, 0


    # Convert payload to velocities
    def payload_to_velocities(self, payload):
        payload = payload.split(" ")

        if payload == "stop":
            return (0.0, 0.0)

        if payload[0] == "return":
            return (0.0, 0.0)

        if payload[0] == "turn":
            if payload[1] == "right":
                return (0.0, -1.0)

            return (0.0, 1.0)

        if payload[0] == "move":
            if payload[1] == "forward":
                return (0.20, 0.0)
            
            return (-0.20, 0.0)
        
        return (0.0, 0.0)

    def is_number(self, number: str) -> bool:
        try:
            float(number)
            return True
        except ValueError:
            return False


    def format_unit(self, value: float, unit: Optional[str]) -> int:
        if not unit:
            return int(round(value))
        if unit in ("m", "meter", "meters"):
            return int(round(value * 100))
        if unit in ("mm", "millimeter", "millimeters"):
            return int(round(value / 10.0))
        return int(round(value))


    def format_number(self, number: str) -> int:
        return int(round(float(number)))


    def format_payload(self, operation: str, *args: Union[str, int]) -> str:
        if not args:
            return operation
        if len(args) == 1:
            return f"{operation} {args[0]}"
        if len(args) == 2 and isinstance(args[0], str) and isinstance(args[1], int):
            return f"{operation} {args[0]} {args[1]}"
        raise TypeError("Invalid format_payload arguments")
