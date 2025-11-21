import threading
import time

import RPi.GPIO as GPIO
import VL53L1X


class ToFSensor:
    def __init__(
        self,
        xshut_front: int = 18,
        xshut_rear: int = 23,
        boot_delay: float = 0.3,
        settle: float = 0.3,
    ):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self._pins = {"front": xshut_front, "rear": xshut_rear}
        self.boot_delay = boot_delay
        self.settle = settle
        self.i2c_bus = 1
        self.i2c_addr = 0x29

        for p in self._pins.values():
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(p, GPIO.LOW)

        self._tofs = {"front": None, "rear": None}
        self._lock = threading.RLock()

    def _enable(self, which):
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.LOW)
        GPIO.output(self._pins[which], GPIO.HIGH)
        time.sleep(self.boot_delay)

    def _disable_both(self):
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.LOW)

    def init_sensor(self, which, range_mode=1):
        """
        Initialize and start ranging for `which` sensor if not already initialized.
        Safe to call multiple times.
        """
        if which not in ("front", "rear"):
            raise ValueError("which must be 'front' or 'rear'")

        with self._lock:
            if self._tofs[which] is not None:
                return

            # Ensure only this sensor is enabled while we open it
            self._enable(which)
            tof = VL53L1X.VL53L1X(i2c_bus=self.i2c_bus, i2c_address=self.i2c_addr)
            tof.open()
            tof.start_ranging(range_mode)
            time.sleep(self.settle)
            self._tofs[which] = tof
            # leave the sensor powered so it can be reused repeatedly

    def close_sensor(self, which):
        """Stop ranging and close the `which` sensor if it is open."""
        if which not in ("front", "rear"):
            raise ValueError("which must be 'front' or 'rear'")

        with self._lock:
            tof = self._tofs.get(which)
            if not tof:
                return
            try:
                tof.stop_ranging()
                tof.close()
            except Exception:
                pass
            self._tofs[which] = None
            # make sure pins are disabled
            self._disable_both()

    def stream(self, which, interval=0.05, range_mode=1, callback=None):
        """
        Blocking continuous stream. Initializes (if needed) and repeatedly reads
        until interrupted with KeyboardInterrupt. If `callback` is provided it will be
        called as callback(which, distance). Otherwise prints readings.
        """
        if which not in ("front", "rear"):
            raise ValueError("which must be 'front' or 'rear'")

        try:
            # ensure sensor is initialized and ranging
            self.init_sensor(which, range_mode=range_mode)
        except Exception:
            return

        print(f"Streaming {which} (Ctrl-C to stop)...")
        try:
            while True:
                with self._lock:
                    tof = self._tofs.get(which)
                    if not tof:
                        distance = None
                    else:
                        try:
                            distance = int(tof.get_distance())
                        except Exception:
                            distance = None
                if callback:
                    try:
                        callback(which, distance)
                    except Exception:
                        # keep streaming even if callback raises
                        pass
                else:
                    print(f"{which}: {distance}")
                time.sleep(interval)
        except KeyboardInterrupt:
            # graceful stop on user interrupt
            pass

    # Background threaded stream
    def _stream_worker(self, which, interval, range_mode, callback, stop_event):
        try:
            # ensure sensor is initialized and ranging
            self.init_sensor(which, range_mode=range_mode)
        except Exception:
            return

        while not stop_event.is_set():
            with self._lock:
                tof = self._tofs.get(which)
                if not tof:
                    distance = None
                else:
                    try:
                        distance = int(tof.get_distance())
                    except Exception:
                        distance = None
            if callback:
                try:
                    callback(which, distance)
                except Exception:
                    pass
            else:
                print(f"{which}: {distance}")
            # use wait so we can stop promptly
            stop_event.wait(interval)

    def start_stream(self, which, interval=0.05, range_mode=1, callback=None):
        """
        Start a non-blocking background streaming thread. Only one stream per instance
        is supported; call stop_stream() before starting another.
        """
        if which not in ("front", "rear"):
            raise ValueError("which must be 'front' or 'rear'")

        if self._stream_thread and self._stream_thread.is_alive():
            raise RuntimeError(
                "Stream already running. Stop it before starting a new one."
            )

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


if __name__ == "__main__":
    sensor = ToFSensor()

    try:
        sensor.stream("front", interval=0.3)
    finally:
        sensor.cleanup()
