import threading
import time
import sys
import RPi.GPIO as GPIO
import VL53L1X

class Matrix:
    def __init__(self):
        self.matrix = [[0.0 for _ in range(16)] for _ in range(16)]
        print("Custom struct 'Matrix' initialized.")
    
    def __list__(self, row, col):
        return self.matrix[row][col]
    
    def __str__(self):
        print("\nMatrix Data:")
        for row in self.matrix:
            for col in row:
                print(f"{col:.2f}", end=" ")
        print("\n")

class TOF_Sensor:
    # Variables
    I2C_BUS = 1
    I2C_ADDR = 0x29
    GRID_SIZE = 16                # 16x16 scan (consider 8 for speed)
    THRESH_MM = 300               # 300 mm threshold
    INTER_MEASUREMENT_MS = 20     # Sensor timing budget interplay; tune as needed
    SLEEP_BETWEEN_ZONES = 0.0     # Small delay after set_user_roi (0–5 ms typically
    STOP_THRESHOLD = 0.5  # distance in meters to trigger stop
    LED_PIN = 18  # GPIO/BCM pin 18 (physical pin 12)
   

    def __init__(self, logger):
        self.logger = logger

        # initialize the sensor
        self.matrix = Matrix()
        GPIO.setmode(GPIO.BOARD)    # or GPIO.BOARD for physical numbering

        GPIO.setup(self.LED_PIN, GPIO.OUT)

        self.blink_led()

        self.floor_distance = self.get_floor_distance()

        print("[TOF Sensor initialized]")


    def make_roi(top, left, bottom, right):
        pass

 
    def get_floor_distance(self) -> float:
        # return the distance to the floor from the sensor once on initialization
        data = self.get_data()
        return data(15,8)  # assuming the floor distance is at the center of the last row

        for p in self._pins.values():
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(p, GPIO.LOW)


    def get_data(self) -> Matrix:
        return self.matrix # Change this to return actual sensor data

        # Streaming attributes
        self._stream_thread = None  # initialize stream attributes
        self._stream_stop = None
        self._stream_which = None
        self._stream_interval = None
        self._stream_range_mode = None
        self._stream_callback = None

    def blink_led(self):
        try:
            while True:
                GPIO.output(self.LED_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(self.LED_PIN, GPIO.LOW)
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            GPIO.cleanup()

        with self._lock:
            if self._tofs[which] is not None:
                return

    def object_detection(self):
        # continuously monitor the sensor data for object detection
        data = self.get_data()
        for value in data: # Only check top 4 rows
            if value < self.STOP_THRESHOLD:
                return True
        for value in data: #check floor distance
            if value < self.floor_distance - 0.1: #if an object is detected closer than the floor distance minus a small margin
                return True

        return False


        # Save stream parameters so we can reuse them when switching sensors
        self._stream_which = which
        self._stream_interval = float(interval)
        self._stream_range_mode = int(range_mode)
        self._stream_callback = callback

        stop_event = threading.Event()
        thread = threading.Thread(
            target=self._stream_worker,
            args=(which, float(interval), int(range_mode), callback, stop_event),
            daemon=True,
        )
        self._stream_thread = thread
        self._stream_stop = stop_event
        thread.start()

    def stop_stream(self, timeout=1.0):
        """Signal the background stream to stop and wait for it to finish."""
        if self._stream_stop:
            self._stream_stop.set()
        if self._stream_thread:
            self._stream_thread.join(timeout=timeout)
        self._stream_thread = None
        self._stream_stop = None
        self._stream_which = None
        self._stream_interval = None
        self._stream_range_mode = None
        self._stream_callback = None

    #
    def switch_sensor(self, which):
        """
        Switch the currently-running sensor to `which`.
        - If a background stream is running, it will be stopped and restarted on `which`
        with the same parameters (interval, range_mode, callback).
        - If no stream is running, this will initialize `which` (and close the other sensor).
        """
        if which not in ("front", "rear"):
            raise ValueError("which must be 'front' or 'rear'")

        other = "rear" if which == "front" else "front"

        # If requested sensor already open and streaming, nothing to do
        if (
            self._stream_thread
            and self._stream_thread.is_alive()
            and self._stream_which == which
        ):
            return

        # Save current stream params (if any)
        current_interval = self._stream_interval
        current_range_mode = self._stream_range_mode
        current_callback = self._stream_callback
        was_streaming = bool(self._stream_thread and self._stream_thread.is_alive())

        # Stop current stream (if any)
        if was_streaming:
            self.stop_stream()

        # Close the other sensor to ensure only `which` is enabled (and to avoid I2C address conflict)
        try:
            self.close_sensor(other)
        except Exception:
            pass

        # Initialize requested sensor (so it's ready)
        try:
            # prefer explicit range_mode if provided earlier; otherwise default to 1
            self.init_sensor(which, range_mode=current_range_mode or 1)
        except Exception:
            # if init fails, leave state consistent and return
            return

        # If we were streaming before, restart streaming on the requested sensor with same params
        if was_streaming:
            interval = current_interval if current_interval is not None else 0.05
            range_mode = current_range_mode if current_range_mode is not None else 1
            callback = current_callback
            # start_stream will set _stream_which etc.
            self.start_stream(
                which, interval=interval, range_mode=range_mode, callback=callback
            )

    #
    def cleanup(self):
        """Close any open sensors, disable pins, and clean up GPIO."""
        try:
            self.stop_stream()
        except Exception:
            pass
        try:
            self.close_sensor("front")
        except Exception:
            pass
        try:
            self.close_sensor("rear")
        except Exception:
            pass
        self._disable_both()
        try:
            GPIO.cleanup()
        except Exception:
            pass

    def cliff_detection(self):
        pass


"""
if __name__ == "__main__":
    sensor = ToFSensor()
    try:
        TOF_Sensor(None)
        print("TOF Sensor initialized and running.")
    except KeyboardInterrupt:
        print("Shutting down TOF Sensor...")
    finally:
        GPIO.cleanup()
