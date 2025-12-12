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

        # Create ROI for ledge and distance.
        self.roi_distance = VL53L1X.VL53L1xUserRoi(6, 3, 9, 0)
        self.roi_distance_wide = VL53L1X.VL53L1xUserRoi(4, 3, 11, 0)
        self.roi_distance_wider = VL53L1X.VL53L1xUserRoi(2, 3, 13, 0)
        self.roi_distance_widest = VL53L1X.VL53L1xUserRoi(0, 3, 15, 0)

        self.roi_ledge = VL53L1X.VL53L1xUserRoi(6, 15, 9, 12)
        self.roi_ledge_wide = VL53L1X.VL53L1xUserRoi(4, 15, 11, 12)
        self.roi_ledge_wider = VL53L1X.VL53L1xUserRoi(2, 15, 13, 12)
        self.roi_ledge_widest = VL53L1X.VL53L1xUserRoi(0, 15, 15, 12)

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
                return -1, -1

            else:
                # Try to read data from tof sensor
                try:
                    # get ditance to ledge
                    tof.stop_ranging()
                    time.sleep(0.1)
                    tof.set_user_roi(self.roi_ledge)
                    time.sleep(0.05)
                    tof.start_ranging(2)
                    time.sleep(0.06)
                    ledge_distance = int(tof.get_distance())

                    # Get distabce to obstacle
                    tof.stop_ranging()
                    time.sleep(0.1)
                    tof.set_user_roi(self.roi_distance)
                    time.sleep(0.05)
                    tof.start_ranging(2)
                    time.sleep(0.06)
                    distance = int(tof.get_distance())

                    # Check for valid distances
                    if ledge_distance > 0 and distance > 0:
                        return ledge_distance, distance

                except:
                    self.logger.error(f"Failed to read distance on sensor '{which}'")
                    return -1, -1

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
                if distance is None:
                    pass

                ledge_distance, distance = distance

                if callback:
                    callback(which, distance)
                else:
                    self.logger.info(f"{which}: {ledge_distance}mm, {distance}mm")

                # Check if ledge disatnce is to high
                if ledge_distance is not None and ledge_distance > 950:
                    return True

                # Check if distance is under collision range
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

        else:
            self._tofs["front"].stop_ranging()
            GPIO.output(self._pins["front"], GPIO.LOW)
            GPIO.output(self._pins["rear"], GPIO.HIGH)
            time.sleep(0.05)

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

    # Experiment

    def roi_experiment(self, which):
        # Run 10 measurements per ROI configuration on the chosen sensor (`"front"` or `"rear"`)
        # and print results to the terminal.

        # Behaviour:
        # - Ensures the requested sensor is powered using `enable()`.
        # - Iterates through distance and ledge ROIs (narrow -> widest).
        # - For each ROI performs 10 start/get/stop cycles and prints each reading.
        # - Prints a simple summary (valid count and average of valid readings).
        with self._lock:
            tof = self._tofs.get(which)
            if not tof:
                self.logger.error(f"Failed to get sensor '{which}' for ROI experiment")
                return

            # Ensure correct sensor is enabled (reuse existing enable logic)
            # front -> positive linear_vel, rear -> negative
            self.enable(1.0 if which == "front" else -1.0)
            time.sleep(self.settle)

            # Ordered list of ROIs to test
            roi_list = [
                ("distance", self.roi_distance),
                ("distance_wide", self.roi_distance_wide),
                ("distance_wider", self.roi_distance_wider),
                ("distance_widest", self.roi_distance_widest),
                ("ledge", self.roi_ledge),
                ("ledge_wide", self.roi_ledge_wide),
                ("ledge_wider", self.roi_ledge_wider),
                ("ledge_widest", self.roi_ledge_widest),
            ]

            print(f"Starting ROI experiment on '{which}' sensor")
            try:
                for roi_name, roi in roi_list:
                    readings = []
                    print(f"\nROI: {roi_name}")

                    # Make sure no ranging is active, then set ROI once
                    try:
                        tof.stop_ranging()
                    except Exception:
                        # ignore stop errors
                        pass
                    time.sleep(0.1)

                    try:
                        tof.set_user_roi(roi)
                    except Exception as e:
                        self.logger.error(
                            f"Failed to set ROI '{roi_name}' on '{which}': {e}"
                        )
                        print(f"  Failed to set ROI '{roi_name}'")
                        continue

                    time.sleep(0.03)

                    for i in range(10):
                        try:
                            # Start ranging, wait briefly, read, then stop
                            tof.start_ranging(2)
                            time.sleep(0.06)
                            val = int(tof.get_distance())
                        except Exception as e:
                            self.logger.error(
                                f"Read failed for ROI '{roi_name}' ({which}) at iter {i}: {e}"
                            )
                            val = -1
                        finally:
                            # Ensure we stop ranging between measurements
                            try:
                                tof.stop_ranging()
                            except Exception:
                                pass

                        readings.append(val)
                        print(f"  #{i + 1:02d}: {val} mm")
                        # small pause between measurements to give sensor time to settle
                        time.sleep(0.1)

                    # Summary of this ROI
                    valid = [r for r in readings if r > 0]
                    if valid:
                        avg = sum(valid) / len(valid)
                        print(f"  Summary: {len(valid)}/10 valid, avg = {avg:.1f} mm")
                    else:
                        print("  Summary: 0/10 valid readings")

            finally:
                # Make sure ranging is stopped when done
                try:
                    tof.stop_ranging()
                except Exception:
                    pass

                print(f"\nCompleted ROI experiment on '{which}'")


if __name__ == "__main__":
    import logging
    import sys

    # Basic logger for CLI runs
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
    )
    logger = logging.getLogger("ToFSensor")

    sensor = None
    try:
        # Instantiate sensor manager
        sensor = ToFSensor(logger=logger)

        # Run ROI experiment on front sensor
        logger.info("Running ROI experiment on 'front' sensor")
        sensor.roi_experiment("front")

        # Optionally run ROI experiment on rear sensor
        logger.info("Running ROI experiment on 'rear' sensor")
        sensor.roi_experiment("rear")

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.exception("Unhandled exception during ROI experiments: %s", e)
    finally:
        # Ensure sensors are closed and GPIO is cleaned up
        if sensor is not None:
            try:
                sensor.close_all()
            except Exception:
                logger.exception("Error while closing sensors")
        try:
            GPIO.cleanup()
        except Exception:
            logger.exception("Error during GPIO.cleanup()")
        sys.exit(0)
