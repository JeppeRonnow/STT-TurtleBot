import statistics
import threading
import time

import RPi.GPIO as GPIO
import VL53L1X


class ToFSensor:
    def __init__(self, front_xshut=18, rear_xshut=23, boot_delay=0.3, settle=0.1):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self._pins = {"front": front_xshut, "rear": rear_xshut}
        self.boot_delay = float(boot_delay)
        self.settle = float(settle)
        self.i2c_bus = 1
        self.i2c_addr = 0x29

        # Latest measurement + lock for thread safety
        self.latest_distance_mm = None
        self.latest_sensor = None  # "front" or "rear"
        self.latest_timestamp = 0.0
        self._lock = threading.Lock()

        # Hold both in reset at start
        for p in self._pins.values():
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(p, GPIO.LOW)

        # Control flags
        self.running = False
        self.use_front = False  # <-- Toggle this boolean whenever you want

    def _enable(self, which: str):
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.LOW)
        time.sleep(0.001)  # tiny delay to be safe
        GPIO.output(self._pins[which], GPIO.HIGH)
        time.sleep(self.boot_delay)

    def _disable_both(self):
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.LOW)

    def _single_measurement(self, which: str, samples: int = 5) -> int:
        """Take median of several readings from one sensor (sensor is already enabled)"""
        tof = VL53L1X.VL53L1X(i2c_bus=self.i2c_bus, i2c_address=self.i2c_addr)
        tof.open()
        tof.start_ranging(3)  # 3 = long range (up to ~4 m)
        time.sleep(self.settle)

        vals = []
        for _ in range(samples):
            dist = tof.get_distance()
            if dist > 0:  # VL53L1X returns 0 on error/out-of-range sometimes
                vals.append(dist)
            time.sleep(0.033)  # ~30 Hz max rate

        tof.stop_ranging()
        tof.close()

        return int(statistics.median(vals)) if vals else 0

    def _worker(self, interval: float = 0.1):
        """Background thread that continuously updates the latest reading"""
        while self.running:
            sensor = "front" if self.use_front else "rear"

            self._enable(sensor)
            try:
                distance = self._single_measurement(sensor, samples=5)
            except Exception as e:
                distance = None
            finally:
                self._disable_both()

            # Update shared state (thread-safe)
            with self._lock:
                self.latest_distance_mm = distance
                self.latest_sensor = sensor
                self.latest_timestamp = time.time()

            time.sleep(interval)

    def start(self, interval: float = 0.1):
        """Start continuous background measurements"""
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(
            target=self._worker, args=(interval,), daemon=True
        )
        self.thread.start()
        time.sleep(0.5)  # give it time to boot first sensor

    def stop(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join(timeout=1.0)
        self._disable_both()

    def get_distance(self):
        """Thread-safe way to read the latest value (returns tuple)"""
        with self._lock:
            return {
                "sensor": self.latest_sensor,
                "distance_mm": self.latest_distance_mm,
                "age_s": time.time() - self.latest_timestamp
                if self.latest_timestamp
                else None,
            }

    def cleanup(self):
        self.stop()
        try:
            GPIO.cleanup()
        except Exception:
            pass


# ========================
# Example continuous usage
# ========================
if __name__ == "__main__":
    sensor = ToFSensor(front_xshut=18, rear_xshut=23, boot_delay=0.3, settle=0.1)

    try:
        sensor.start(interval=0.15)  # ~6-7 Hz update rate (safe for two sensors)
        print("ToF continuous loop started. Press Ctrl+C to stop.")

        while True:
            data = sensor.get_distance()
            print(
                f"{data['sensor'].upper():5} | {data['distance_mm']:4} mm | "
                f"age {data['age_s']:.3f}s"
                if data["age_s"]
                else "",
                end="\r",
            )

            # Example: toggle every 5 seconds
            if int(time.time()) % 10 < 5:
                sensor.use_front = True
            else:
                sensor.use_front = False

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        sensor.cleanup()
        print("Cleaned up GPIO")
