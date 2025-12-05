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

    # Test Test Test Test Test Test Test Test Test Test Test Test Test Test Test Test Test Test Test Test
    # Edge Detection
    def set_ledge_detect_roi(self, which: str):
        tof = self._tofs.get(which)
        try:
            # brief settle before stopping ranging
            time.sleep(0.05)
            roi = VL53L1X.VL53L1xUserRoi(6, 9, 15, 14)
            tof.stop_ranging()
            time.sleep(0.05)
            tof.set_user_roi(roi)
            # brief settle before starting ranging again
            time.sleep(0.02)
            tof.start_ranging(1)
            # brief settle to let ranging stabilize
            time.sleep(0.05)
            self.logger.info(f"ROI applied on {which}: {roi}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to set ROI on '{which}': {e}")
            return False

    def set_wide_roi(self, which: str):
        tof = self._tofs.get(which)
        try:
            # brief settle before stopping ranging
            time.sleep(0.05)
            roi = VL53L1X.VL53L1xUserRoi(0, 15, 15, 0)
            tof.stop_ranging()
            time.sleep(0.05)
            tof.set_user_roi(roi)
            # brief settle before starting ranging again
            time.sleep(0.02)
            tof.start_ranging(1)
            # brief settle to let ranging stabilize
            time.sleep(0.05)
            self.logger.info(f"ROI applied on {which}: {roi}")
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
        self.set_ledge_detect_roi("front")
        fm, fi, fn = self.sample_median("front")
        self._baseline_front = fm
        self.logger.info(f"Front baseline: {fm} mm (invalid {fi}/{fn})")

        # Rear
        self.enable(-0.1)
        self.set_ledge_detect_roi("rear")
        rm, ri, rn = self.sample_median("rear")
        self._baseline_rear = rm
        self.logger.info(f"Rear baseline: {rm} mm (invalid {ri}/{rn})")

    def run_once(self):
        # Front
        self.enable(0.1)
        fm, fi, fn = self.sample_median("front")
        fb = getattr(self, "_baseline_front", None)
        fdelta = fm - fb if (fm is not None and fb is not None) else None

        # Rear
        self.enable(-0.1)
        rm, ri, rn = self.sample_median("rear")
        rb = getattr(self, "_baseline_rear", None)
        rdelta = rm - rb if (rm is not None and rb is not None) else None

        self.logger.info(
            f"Front: cur={fm} mm base={fb} mm Δ={fdelta} invalid={fi}/{fn}"
        )
        self.logger.info(
            f"Rear:  cur={rm} mm base={rb} mm Δ={rdelta} invalid={ri}/{rn}"
        )

    def ledge_detect(self, should_check_ledge: bool, linear_vel):
        which = "front" if linear_vel > 0.0 else "rear"

        if not should_check_ledge:
            return False, None, 0

        self.enable(linear_vel)
        time.sleep(0.05)
        self.set_ledge_detect_roi(which)
        time.sleep(0.05)

        median, invalid, n = self.sample_median(which, n=3, interval=0.05)

        # Compare against baseline
        base = getattr(
            self, "_baseline_front" if which == "front" else "_baseline_rear", None
        )
        delta = (median - base) if (median is not None and base is not None) else None

        delta_flag = bool(delta is not None and abs(delta) >= 80)
        invalid_flag = invalid >= 2
        flag = delta_flag or invalid_flag

        # Print concise status and return
        self.logger.info(
            f"{which.capitalize()}: cur={median}mm base={base}mm Δ={delta} invalid={invalid}/{n} flag={'YES' if flag else 'NO'}"
        )
        return flag, median, invalid


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
        ts.enable(0.1)
        ts.set_ledge_detect_roi("front")
        fm, fi, fn = ts.sample_median("front")
        print(f"[INFO] Front baseline: {fm} mm (invalid {fi}/{fn})")

        ts.enable(-0.1)
        ts.set_ledge_detect_roi("rear")
        rm, ri, rn = ts.sample_median("rear")
        print(f"[INFO] Rear baseline: {rm} mm (invalid {ri}/{rn})")

        ts.enable(0.1)
        fm, fi, fn = ts.sample_median("front")
        ts.enable(-0.1)
        rm, ri, rn = ts.sample_median("rear")
        print(f"[INFO] Front: cur={fm} mm invalid={fi}/{fn}")
        print(f"[INFO] Rear:  cur={rm} mm invalid={ri}/{rn}")

    except KeyboardInterrupt:
        pass
    finally:
        ts.close_all()
