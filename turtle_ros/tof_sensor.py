import threading
import time
from venv import logger

import RPi.GPIO as GPIO
import VL53L1X


class ToFSensor:
    def __init__(
        self,
        logger,
        i2c_bus: int = 1,
        boot_delay: float = 0.1,
        settle: float = 0.3,
        xshut_front: int = 17,
        xshut_rear: int = 27,
    ):
        self.logger = logger
        self.i2c_bus = i2c_bus
        self.boot_delay = boot_delay
        self.settle = settle
        self._pins = {"front": xshut_front, "rear": xshut_rear}

        # Set unique sensor address
        self.i2c_front_addr = 0x2A

        # Storing sensor class
        self._tofs = {"front": None, "rear": None}

        # Set up threading needs
        self._lock = threading.RLock()
        self.collision_thread = None
        self.collision_thread_flag = threading.Event()

        # Set up GPIO for xshut used for start up
        GPIO.setmode(GPIO.BCM)

        for p in self._pins.values():
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)

        time.sleep(self.boot_delay)

        # Init the two sensors
        self.init_sensor()

        # Used to keep track of direction
        self.direction = ""

    # Start VL53L1X class
    def init_sensor(self):
        # Enable Front sensor
        GPIO.output(self._pins["rear"], GPIO.LOW)
        GPIO.output(self._pins["front"], GPIO.HIGH)  # Power xshut for front sensor
        time.sleep(self.boot_delay)

        # Init front sensor
        tof_front = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof_front.open()
        time.sleep(self.boot_delay)
        self.logger.info("Front sensor initialized")

        # Enable Rear sensor
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.HIGH)
        time.sleep(self.boot_delay)

        # Init sensor
        tof_rear = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof_rear.open()
        self.logger.info("Rear sonsor initialized")

        # Save tof classes
        self._tofs["front"] = tof_front
        self._tofs["rear"] = tof_rear

        # Disable both sensors
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.LOW)

    # Reads data from sensor
    def get_distance(self, which):
        with self._lock:
            # Get sensor
            tof = self._tofs.get(which)

            # Check if sensor class is active
            if not tof:
                self.logger.error(f"Failed to get sensor '{which}'")
                return None

            else:
                # Try to read data from tof sensor
                try:
                    distance = int(tof.get_distance())
                    return distance if distance > 0 else None

                except:
                    self.logger.error(f"Failed to read distance on sensor '{which}'")
                    return None

    # Continuous sensor read in a blocking way
    def stream(self, which, interval=0.05, callback=None):
        # Clear flag
        self.collision_thread_flag.clear()

        try:
            while True:
                # Break if signaled to stop
                if self.collision_thread_flag.is_set():
                    break

                # Read distance from sensor
                distance = self.get_distance(which)

                if callback:
                    callback(which, distance)
                else:
                    self.logger.info(f"{which}: {distance}mm")

                # Check if distance is with in collision range
                if distance is not None and distance < 300:
                    return True

                time.sleep(interval)  # Wait for reading interval

        except KeyboardInterrupt:
            pass

    # Enable reading on desired sensor
    def enable(self, linear_vel):
        # Check which sensor to enable
        if linear_vel > 0.0:
            direction = "front"
        else:
            direction = "rear"

        # Check if direction is already set
        if self.direction == direction:
            return

        # Save the new direction
        self.direction = direction
        self.logger.info(f"Switching to {direction} sensor")

        # Enable desired sensor
        if direction == "front":
            self._tofs["rear"].stop_ranging()
            GPIO.output(self._pins["rear"], GPIO.LOW)
            GPIO.output(self._pins["front"], GPIO.HIGH)
            time.sleep(0.05)
            self._tofs["front"].start_ranging(1)

        else:
            self._tofs["front"].stop_ranging()
            GPIO.output(self._pins["front"], GPIO.LOW)
            GPIO.output(self._pins["rear"], GPIO.HIGH)
            time.sleep(0.05)
            self._tofs["rear"].start_ranging(1)

    # Close a given sensor
    def close_sensor(self, which):
        with self._lock:
            # Get tof class
            tof = self._tofs.get(which)

            # Check if tof class is active
            if not tof:
                self.logger.error(f"{which} is not active")
                return

            # Try to close tof sensor
            try:
                tof.stop_ranging()
                tof.close()

            except Exception:
                self.logger.error(f"Failed to stop and close '{which}'")

            # Clear sensor class
            self._tofs[which] = None

    # Close all sensors
    def close_all(self):
        self.close_sensor("front")
        self.close_sensor("rear")

    # Ledge Detection
    #
    # Left, right, top and bottom are relative to the SPAD matrix coordinates,
    # which will be mirrored in real scene coordinates.
    # (or even rotated, depending on the VM53L1X element alignment on the board and on the board position)
    #
    # ROI in SPAD matrix coords:
    #
    # 15  top-left
    # |  X____
    # |  |    |
    # |  |____X
    # |        bottom-right
    # 0__________15

    def set_roi(self, roi_preset: str):
        which = self.direction if self.direction in ("front", "rear") else "front"
        tof = self._tofs.get(which)

        if roi_preset == "ledge_detect":
            roi = VL53L1X.VL53L1xUserRoi(7, 8, 15, 14)
        elif roi_preset == "default":
            roi = VL53L1X.VL53L1xUserRoi(0, 15, 9, 0)
        else:
            roi = VL53L1X.VL53L1xUserRoi(0, 15, 15, 0)

        try:
            # brief settle before stopping ranging
            time.sleep(0.05)
            tof.stop_ranging()
            time.sleep(0.05)
            tof.set_user_roi(roi)
            # brief settle before starting ranging again
            time.sleep(0.02)
            tof.start_ranging(1)
            # brief settle to let ranging stabilize
            time.sleep(0.05)
            self.logger.info(f"ROI applied on {which}: {roi_preset}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to set ROI on '{which}': {e}")
            return False

    def sample_median(self, which: str, n: int = 3, interval: float = 0.05):
        values = []
        invalid = 0
        for _ in range(n):
            d = self.get_distance(which)
            if d is None:
                invalid += 1
            else:
                values.append(d)
            time.sleep(interval)
        median = None
        if values:
            values.sort()
            median = values[len(values) // 2]
        return median, invalid, n

    def calibrate_baseline(self):
        # Front
        self.enable(0.1)
        time.sleep(0.1)
        self.set_roi("ledge_detect")
        time.sleep(0.1)
        fm, fi, fn = self.sample_median("front", 3, 0.05)
        self._baseline_front = fm
        self.logger.info(f"Front baseline: {fm} mm (invalid {fi}/{fn})")

        # Rear
        time.sleep(0.1)
        self.enable(-0.1)
        time.sleep(0.1)
        self.set_roi("ledge_detect")
        time.sleep(0.1)
        rm, ri, rn = self.sample_median("rear", 3, 0.05)
        self._baseline_rear = rm
        self.logger.info(f"Rear baseline: {rm} mm (invalid {ri}/{rn})")

    def ledge_detect(
        self,
        should_check_ledge: bool,
        linear_vel,
        delta_threshold_mm: int = 80,
        invalid_threshold: int = 2,
        debounce_cycles: int = 2,
        interval: float = 0.05,
    ):
        # Decide sensor by velocity sign
        which = "front" if linear_vel > 0.0 else "rear"

        if not should_check_ledge:
            return False, None, 0

        # Enable and apply ROI once
        self.enable(linear_vel)
        time.sleep(0.05)
        self.set_roi("ledge_detect")
        time.sleep(0.05)

        # Retrieve baseline
        base = getattr(
            self, "_baseline_front" if which == "front" else "_baseline_rear", None
        )
        if base is None:
            self.logger.error(
                f"No baseline set for {which}. Call calibrate_baseline() first."
            )
            return False, None, 0

        # Continuous detection loop with debounce; stop when collision_thread_flag is set
        streak = 0
        flag = False
        while not self.collision_thread_flag.is_set():
            median, invalid, n = self.sample_median(which, 3, interval)

            delta = (median - base) if (median is not None) else None
            delta_flag = bool(delta is not None and abs(delta) >= delta_threshold_mm)
            invalid_flag = invalid >= invalid_threshold
            current_flag = delta_flag or invalid_flag

            streak = streak + 1 if current_flag else 0
            flag = streak >= debounce_cycles

            # Print concise status
            self.logger.info(
                f"{which.capitalize()}: cur={median}mm base={base}mm Δ={delta} invalid={invalid}/{n} "
                f"delta_flag={'YES' if delta_flag else 'NO'} invalid_flag={'YES' if invalid_flag else 'NO'} "
                f"flag={'YES' if flag else 'NO'} streak={streak}/{debounce_cycles}"
            )

            # Keep running continuously even if flag is true (per your test approach)
            time.sleep(interval)

        return (
            flag,
            median if "median" in locals() else None,
            invalid if "invalid" in locals() else 0,
        )


if __name__ == "__main__":
    logger = type(
        "Log",
        (),
        {
            "info": lambda self, msg: print(msg),
            "error": lambda self, msg: print(msg),
        },
    )()

    # Instantiate sensor manager
    ts = ToFSensor(logger, 1, 0.1, 0.3, 17, 27)

    try:
        ts.calibrate_baseline()
        time.sleep(0.3)
        ts.ledge_detect(True, -0.1)

    except KeyboardInterrupt:
        pass
    finally:
        ts.close_all()
