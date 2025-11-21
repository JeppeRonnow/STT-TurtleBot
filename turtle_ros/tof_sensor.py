import statistics
import time

import RPi.GPIO as GPIO
import VL53L1X


class ToFSensor:
    def __init__(self, front_xshut=18, rear_xshut=23, boot_delay=0.3, settle=0.3):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self._pins = {"front": front_xshut, "rear": rear_xshut}
        self.boot_delay = float(boot_delay)
        self.settle = float(settle)
        self.i2c_bus = 1
        self.i2c_addr = 0x29

        # ensure pins exist and start held in reset (LOW)
        for p in self._pins.values():
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(p, GPIO.LOW)

    def _enable(self, which):
        # hold both reset, then enable requested sensor
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.LOW)
        GPIO.output(self._pins[which], GPIO.HIGH)
        time.sleep(self.boot_delay)

    def _disable_both(self):
        GPIO.output(self._pins["front"], GPIO.LOW)
        GPIO.output(self._pins["rear"], GPIO.LOW)

    def measure(self, which, samples=3, delay_between=0.1, range_mode=3):
        """Enable sensor, take `samples` readings, disable both, return median (int mm) or None."""
        if which not in ("front", "rear"):
            raise ValueError("which must be 'front' or 'rear'")

        self._enable(which)
        tof = None
        vals = []
        try:
            tof = VL53L1X.VL53L1X(i2c_bus=self.i2c_bus, i2c_address=self.i2c_addr)
            tof.open()
            tof.start_ranging(range_mode)
            time.sleep(self.settle)
            for _ in range(max(1, int(samples))):
                try:
                    vals.append(int(tof.get_distance()))
                except Exception:
                    # ignore individual read errors
                    pass
                time.sleep(delay_between)
        except Exception:
            vals = []
        finally:
            try:
                if tof:
                    tof.stop_ranging()
                    tof.close()
            except Exception:
                pass
            self._disable_both()

        if not vals:
            return None
        return int(statistics.median(vals))

    def measure_front(self, samples=3, delay_between=0.1, range_mode=1):
        return self.measure(
            "front", samples=samples, delay_between=delay_between, range_mode=range_mode
        )

    def measure_rear(self, samples=3, delay_between=0.1, range_mode=1):
        return self.measure(
            "rear", samples=samples, delay_between=delay_between, range_mode=range_mode
        )

    def cleanup(self):
        """Disable sensors and cleanup GPIO."""
        self._disable_both()
        try:
            GPIO.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    dp = ToFSensor(front_xshut=18, rear_xshut=23, boot_delay=0.3, settle=0.1)
    try:
        time.sleep(0.1)
        print("Front:", dp.measure_front())
        time.sleep(0.1)
        print("Rear: ", dp.measure_rear())
    finally:
        dp.cleanup()
