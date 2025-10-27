from typing import List, Optional, Tuple, overload, Union

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
    def __init__(self, PAUSE_ITTERATIONS, DEFAULT_TURN_DEG, DEFAULT_DISTANCE_CM, DEBUG) -> None:
        self.PAUSE_ITTERATIONS = PAUSE_ITTERATIONS
        self.DEFAULT_TURN_DEG = DEFAULT_TURN_DEG
        self.DEFAULT_DISTANCE_CM = DEFAULT_DISTANCE_CM
        self.DEBUG = DEBUG
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
                direction = ""

                # Use the remaining words starting from current position
                for j in range(i, n):
                    if words[j] in self.dir_syn:
                        direction = words[j]
                   
                payload = self.format_payload("turn", direction)
                return payload, i + 1

            # Move forwards
            if word in self.fwd_syn:
                payload = self.format_payload("move", "forward")
                return payload, i + 1
            
            # Move backwards
            if word in self.back_syn:
                payload = self.format_payload("move", "forward")
                return payload, i + 1


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
