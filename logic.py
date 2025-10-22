from typing import List, Optional, Tuple, overload, Union

import config

move_syn = {"move", "go", "walk", "drive"}
turn_syn = {"turn", "rotate"}
stop_syn = {"stop", "halt", "freeze", "wait", "pause"}
return_syn = {"return", "back"}

dir_syn = {"left", "right"}
fwd_syn = {"forward", "straight", "ahead"}
back_syn = {"backward", "backwards", "reverse"}

deg_units = {"deg", "degree", "degrees"}
dist_units = {"mm", "millimeter", "millimeters", "cm", "centimeter", "centimeters", "m", "meter", "meters"}

def handle_transcription(words: List[str]) -> Tuple[Optional[str], int]:
    global move_syn, turn_syn, stop_syn, return_syn, dir_syn, fwd_syn, back_syn, deg_units, dist_units, pause

    if not words:
        return None, None

    n = len(words)
    for i, word in enumerate(words):
        # Stop
        if word in stop_syn:
            operation = "stop"
            return operation, i + 1

        # Return
        if word in return_syn:
            operation = "return"
            return operation, i + 1

        # Turn left/right
        if word in turn_syn:
            operation = "turn"
            direction = None
            distance = None
            last_word_index = 0
            for i, word in enumerate(words):
                if word in dir_syn:
                    direction = word
                    if i + 1 > last_word_index: 
                        last_word_index = i + 1
                if _is_number(word):
                    distance = format_number(word)
                    if i + 1 > last_word_index: 
                        last_word_index = i + 1
                if direction and distance:
                    break
            if last_word_index >= n and not distance and config.CURRENT_PAUSE < config.PAUSE_ITTERATIONS:
                continue
            if not distance:
                distance = config.DEFAULT_TURN_DEG
            if direction and distance:
                consumed = last_word_index + 1
                payload = format_payload(operation ,direction, distance)
                return payload, consumed

        # Move
        if word in fwd_syn or word in back_syn:
            operation = "move"
            direction = "forward" if word in fwd_syn else "backward"
            distance = None
            unit = None
            last_word_index = 0
            for i, word in enumerate(words):
                if _is_number(word):
                    distance = format_number(word)
                    if i + 1 > last_word_index: 
                        last_word_index = i + 1
                if word in dist_units:
                    unit = word
                    if i + 1 > last_word_index: 
                        last_word_index = i + 1
                if distance and unit:
                    break
            if last_word_index >= n and not distance and not unit and config.CURRENT_PAUSE < config.PAUSE_ITTERATIONS:
                continue
            if not distance:
                distance = config.DEFAULT_DISTANCE_CM
            if unit:
                distance = format_unit(distance, unit)
            if distance:
                consumed = last_word_index + 1
                payload = format_payload(operation ,direction, distance)
                return payload, consumed

    config.CURRENT_PAUSE += 1
    return None, None

def _is_number(number: str) -> bool:
    try:
        float(number)
        return True
    except ValueError:
        return False
    
def format_unit(value: float, unit: Optional[str]) -> int:
    if not unit:
        return int(round(value))
    if unit in ("m", "meter", "meters"):
        return int(round(value * 100))
    if unit in ("mm", "millimeter", "millimeters"):
        return int(round(value / 10.0))
    return int(round(value))

def format_number(number: str) -> int:
    return int(round(float(number)))

@overload
def format_payload(operation: str) -> str: ...
@overload
def format_payload(operation: str, direction: str) -> str: ...
@overload
def format_payload(operation: str, distance: int) -> str: ...
@overload
def format_payload(operation: str, direction: str, distance: int) -> str: ...

def format_payload(operation: str, *args: Union[str, int]) -> str:
    # 0 args -> "op"
    if not args:
        return operation
    # 1 arg -> "op X" (direction or distance)
    if len(args) == 1 and isinstance(args[0], (str, int)):
        return f"{operation} {args[0]}"
    # 2 args -> "op direction distance"
    if len(args) == 2 and isinstance(args[0], str) and isinstance(args[1], int):
        return f"{operation} {args[0]} {args[1]}"
    raise TypeError("Invalid format_payload arguments")