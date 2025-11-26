import threading
import time
import RPi.GPIO as GPIO
from VL53L1X import VL53L1X


class ToFSensor:
    def __init__(
        self,
        logger,
        i2c_bus: int = 1,
        boot_delay: float = 0.1,
        settle: float = 0.3,
        xshut_front: int = 17,
        xshut_rear: int = 27
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


    # Start VL53L1X class
    def init_sensor(self):
        # Enable Front sensor
        GPIO.output(self._pins["rear"], GPIO.LOW)
        GPIO.output(self._pins["front"], GPIO.HIGH) # Power xshut for front sensor
        time.sleep(self.boot_delay)

        # Init front sensor
        tof_front = VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof_front.open()
        time.sleep(self.boot_delay)
        self.logger.info("Front sensor initialized")

        # Enable Rear sensor
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.HIGH)
        time.sleep(self.boot_delay)

        # Init sensor
        tof_rear = VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof_rear.open()
        self.logger.info("Rear sonsor initialized")

        # Save tof classes
        self._tofs["front"] = tof_front
        self._tofs["rear"] = tof_rear


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
    def stream(self, which, interval=0.3, callback=None):
        # Clear flag
        self.collision_thread_flag.clear()

        self.enable(which)

        try:
            while True:
                # Break if signaled to stop
                if self.collision_thread_flag.is_set():
                    break;

                # Read distance from sensor
                distance = self.get_distance(which)

                if callback:
                    callback(which, distance)
                else:
                    self.logger.info(f"{which}: {distance}mm")

                # Check if distance is with in collision range
                if distance is not None and distance < 100:
                    return True

                time.sleep(interval) # Wait for reading interval

        except KeyboardInterrupt:
            pass


    # Enable reading on desired sensor
    def enable(self, which):
        if which == "front":
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
                self.logger.error(F"{which} is not active")
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
