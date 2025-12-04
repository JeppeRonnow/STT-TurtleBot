import logging
import time
from random import sample
from typing import List, Optional
import VL53L1X
from tof_sensor import ToFSensor

SCAN_PRESETS = {"bottom_wide": (0, 15, 15, 15), "bottom_narrow": (0, 7, 2, 2)}


class EdgeDetection(ToFSensor):
    def set_scan_range(self, tof, scan_preset: str) -> bool:
        scan_args = SCAN_PRESETS.get(scan_preset)

        # Check if scan preset available
        if scan_args is None:
            self.logger.error(f"Unknown scan_preset '{scan_preset}'")
            return False

        # Check if the VL53L1xUserRoi class is available in the VL53L1X module
        if not hasattr(VL53L1X, "VL53L1xUserRoi"):
            self.logger.error("VL53L1xUserRoi not available in VL53L1X module")
            return False

        # Construct scan object
        try:
            scan_obj = VL53L1X.VL53L1xUserRoi(*scan_args)
        except Exception as e:
            self.logger.error(f"Failed to construct VL53L1xUserRoi: {e}")
            return False

        # Stop ranging when possible to avoid conflicts, but it's not fatal if it fails.
        try:
            tof.stop_ranging()
        except Exception:
            # Some wrappers allow set_user_roi while ranging; continue.
            pass

        # Apply the scan prest and return success status.
        try:
            tof.set_user_roi(scan_obj)
        except Exception as e:
            self.logger.error(f"set_user_roi(scan_obj) failed: {e}")
            try:
                tof.start_ranging(1)
            except Exception:
                pass
            return False

        try:
            tof.start_ranging(3)
        except Exception:
            pass

        return True

    def capture_floor_distance(
        self, which: str, samples: int, delay: float, scan_preset: str
    ) -> List[Optional[int]]:
        tof = self._tofs.get(which)

        if tof is None:
            self.logger.error(f"Sensor {which} not found")
            return []

        if not self.set_scan_range(tof, scan_preset):
            self.logger.error("Failed to apply scan preset; aborting ")
            return []

        samples_collected: List[Optional[int]] = []

        for i in range(samples):
            try:
                d = tof.get_distance()
                samples_collected.append(
                    None if (d == 0 or d is None or d >= 500) else int(d)
                )
            except Exception:
                self.logger.debug("get_distance() failed")
                samples_collected.append(None)
            time.sleep(delay)

        try:
            tof.start_ranging(3)
        except Exception:
            pass

        filtered = [s for s in samples_collected if s is not None]
        return filtered if filtered else samples_collected

    def determine_default_distance(
        self,
        which: str,
        scan_preset: str = "bottom_wide",
        baseline_samples: int = 3,
        sample_delay: float = 0.1,
    ) -> Optional[dict]:
        # Determine baseline for rear
        try:
            self.enable("rear")
            time.sleep(self.settle)
        except Exception:
            tof = self._tofs.get()
            if tof:
                try:
                    tof.start_ranging(1)
                    time.sleep(self.settle)
                except Exception:
                    pass

        samples = self.capture_floor_distance(
            which,
            baseline_samples,
            sample_delay,
            scan_preset,
        )
        if not samples:
            self.logger.error("Failed to capture baseline samples")
            return None

        # Compute the mean over valid readings only.
        valid = [s for s in samples if s is not None]
        if not valid:
            self.logger.error("No valid baseline readings obtained")
            return None

        try:
            baseline_mean = mean(valid)
        except Exception:
            self.logger.error("Failed to compute baseline mean")
            return None

        return {"baseline_mean": baseline_mean, "samples": samples}
        # Determine baseline front


    def run_edge_detection(
        self,
        which: str = "rear",
        scan_preset: str = "bottom_wide",
        baseline: Optional[dict] = None,
        threshold_mm: int = 200,
        consecutive_required: int = 3,
        check_samples: int = 3,
        check_delay: float = 0.05,
        interval: float = 1.0,
        timeout: Optional[float] = None,
        callback: Optional[Callable[[bool, dict], None]] = None,
    ) -> dict:
        start_time = time.time()
        checks = 0
        consecutive_hits = 0

        # Use provided baseline or compute one now.
        if baseline is None:
            baseline = self.determine_default_distance(which=which, scan_preset=scan_preset, baseline_samples=20, sample_delay=0.05)
            if baseline is None:
                return {"edge_detected": None, "reason": "failed_to_establish_baseline"}

        baseline_mean = baseline["baseline_mean"]

        try:
            self.enable(which)
        except Exception:
            tof = self._tofs.get(which)
            if tof:
                try:
                    tof.start_ranging(3)
                except Exception:
                    pass

        # Monitoring loop: sample ROI, compute mean, compare to baseline.
        try:
            while True:
                if timeout is not None and (time.time() - start_time) > timeout:
                    return {
                        "edge_detected": False,
                        "baseline_mean": baseline_mean,
                        "checks": checks,
                        "reason": "timeout",
                    }

                samples = self.capture_floor_distance(which, check_samples, check_delay, scan_preset)
                checks += 1

                if not samples:
                    self.logger.debug("No samples collected this check; skipping")
                    time.sleep(interval)
                    continue

                valid = [s for s in samples if s is not None]
                if not valid:
                    self.logger.debug("All samples invalid this check; skipping")
                    time.sleep(interval)
                    continue

                current_mean = mean(valid)
                diff = abs(current_mean - baseline_mean)
                self.logger.info(f"Check {checks}: current_mean={current_mean} mm baseline_mean={baseline_mean} mm diff={diff} mm")

                # Determine if current check constitutes a significant height change.
                if diff >= threshold_mm:
                    consecutive_hits += 1
                    if consecutive_hits >= consecutive_required:
                        result = {
                            "edge_detected": True,
                            "baseline_mean": baseline_mean,
                            "current_mean": current_mean,
                            "diff": diff,
                            "checks": checks,
                            "consecutive_required": consecutive_required,
                        }
                        # Invoke callback with True flag and result details.
                        if callback:
                            try:
                                callback(True, result)
                            except Exception:
                                self.logger.debug("Callback raised an exception")
                        return result
                else:
                    consecutive_hits = 0

                time.sleep(interval)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    edge_detector = EdgeDetection()
    result = edge_detector.run_edge_detection(
        which="rear",
        scan_preset="bottom_wide",
        threshold_mm=150,
        consecutive_required=2,
        check_samples=5,
        check_delay=0.1,
        interval=0.5,
        timeout=30.0,
    )
    print("Edge Detection Result:", result)
