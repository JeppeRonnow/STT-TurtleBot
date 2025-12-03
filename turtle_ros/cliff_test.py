import logging
import time
from statistics import mean
from typing import List, Optional, Tuple

import VL53L1X
from tof_sensor import ToFSensor

"""
Cliff detection (ROI-only)

Logic:
1. Set ROI to top (which is the bottom row since sensors are upside down)
2. Establish an ROI-only baseline while the robot is on a surface
3. While driving, sample ROI periodically and compute an ROI mean
4. If the ROI mean deviates from the ROI baseline by >= threshold_mm for
   `consecutive` checks, declare a cliff
"""

ROI_PRESETS = {
    "top": (0, 15, 15, 15),
    "middle": (7, 15, 15, 15),
    "bottom": (15, 15, 15, 15),
}


class CliffTester(ToFSensor):
    """
    Cliff detection using only ROI (no center/primary usage).
    """

    def _apply_roi(self, tof, roi_preset: str) -> bool:
        roi_args = ROI_PRESETS.get(roi_preset)
        if roi_args is None:
            self.logger.error(f"Unknown roi_preset '{roi_preset}'")
            return False

        # Try several common ROI APIs
        try:
            if hasattr(VL53L1X, "VL53L1xUserRoi"):
                try:
                    roi_obj = VL53L1X.VL53L1xUserRoi(*roi_args)
                    try:
                        tof.stop_ranging()
                    except Exception:
                        pass
                    tof.set_user_roi(roi_obj)
                    return True
                except Exception as e:
                    self.logger.debug(f"ROI object approach failed: {e}")
        except Exception:
            pass

        if hasattr(tof, "set_user_roi"):
            try:
                try:
                    tof.stop_ranging()
                except Exception:
                    pass
                tof.set_user_roi(*roi_args)
                return True
            except Exception as e:
                self.logger.debug(f"set_user_roi(*args) failed: {e}")
                try:
                    tof.set_user_roi(tuple(roi_args))
                    return True
                except Exception as e2:
                    self.logger.debug(f"set_user_roi(tuple) failed: {e2}")

        alt_names = ("set_roi", "setUserRoi", "user_roi", "set_user_roi_area")
        for name in alt_names:
            if hasattr(tof, name):
                try:
                    try:
                        tof.stop_ranging()
                    except Exception:
                        pass
                    getattr(tof, name)(*roi_args)
                    return True
                except Exception as e:
                    self.logger.debug(f"{name}(*args) failed: {e}")

        self.logger.error(
            "Unable to set ROI: wrapper doesn't expose a known ROI API. "
            "Inspect the VL53L1X wrapper on your Pi (dir(VL53L1X) and dir(tof)) and tell me the available methods."
        )
        return False

    def _sample_roi_via_tof(
        self, which: str, samples: int, delay: float, roi_preset: str
    ) -> List[Optional[int]]:
        tof = self._tofs.get(which)
        if tof is None:
            self.logger.error(f"Sensor '{which}' not found for ROI sampling")
            return []

        if not self._apply_roi(tof, roi_preset):
            return []

        try:
            tof.start_ranging(3)
        except Exception:
            pass

        samples_collected: List[Optional[int]] = []
        for _ in range(samples):
            try:
                d = tof.get_distance()
                samples_collected.append(None if (d == 0 or d is None) else int(d))
            except Exception:
                self.logger.debug("get_distance() failed while sampling ROI")
                samples_collected.append(None)
            time.sleep(delay)

        try:
            tof.start_ranging(3)
        except Exception:
            pass

        filtered = [s for s in samples_collected if s is not None]
        return filtered if filtered else samples_collected

    def establish_roi_baseline(
        self,
        which: str = "rear",
        roi_preset: str = "top",
        baseline_samples: int = 20,
        sample_delay: float = 0.05,
    ) -> Optional[dict]:
        """
        Establish an ROI-only baseline.

        Returns dict:
          - 'roi_mean'
          - 'roi_samples'
        Or None if baseline could not be obtained.
        """
        self.logger.info("Establishing ROI baseline for cliff detection...")

        # Ensure sensor running
        try:
            self.enable(which)
            time.sleep(self.settle)
        except Exception:
            tof = self._tofs.get(which)
            if tof:
                try:
                    tof.start_ranging(3)
                    time.sleep(self.settle)
                except Exception:
                    pass

        roi_samples = self._sample_roi_via_tof(
            which, baseline_samples, sample_delay, roi_preset
        )
        if not roi_samples:
            self.logger.error("Failed to collect ROI baseline samples")
            return None

        try:
            roi_vals = [r for r in roi_samples if r is not None]
            roi_mean = mean(roi_vals) if roi_vals else None
        except Exception:
            roi_mean = None

        if roi_mean is None:
            self.logger.error("ROI baseline mean could not be computed")
            return None

        self.logger.info(
            f"ROI baseline established: mean={roi_mean} mm from {len(roi_samples)} samples"
        )
        return {"roi_mean": roi_mean, "roi_samples": roi_samples}

    def monitor_for_cliff(
        self,
        which: str = "rear",
        roi_preset: str = "top",
        baseline: Optional[dict] = None,
        baseline_samples: int = 20,
        baseline_delay: float = 0.05,
        check_samples: int = 3,
        check_delay: float = 0.05,
        interval: float = 1.0,
        threshold_mm: int = 200,
        consecutive: int = 3,
        timeout: Optional[float] = None,
    ) -> dict:
        """
        Monitor ROI vs baseline and return when a cliff is detected or timeout occurs.

        Returns dict with:
          - 'baseline_roi_mean'
          - 'current_roi_mean' (when cliff detected or last check)
          - 'diff'
          - 'is_cliff' True/False
          - 'checks' number of checks performed
          - 'consecutive_required'
        """
        start_time = time.time()
        checks = 0
        consec = 0

        # Obtain or use provided baseline
        if baseline is None:
            baseline = self.establish_roi_baseline(
                which=which,
                roi_preset=roi_preset,
                baseline_samples=baseline_samples,
                sample_delay=baseline_delay,
            )
            if baseline is None:
                return {"is_cliff": None, "reason": "failed_to_establish_baseline"}

        baseline_mean = baseline["roi_mean"]

        self.logger.info(
            f"Starting ROI monitoring: baseline_mean={baseline_mean} mm, threshold={threshold_mm} mm, consecutive={consecutive}"
        )

        try:
            # Ensure sensor enabled
            try:
                self.enable(which)
            except Exception:
                tof = self._tofs.get(which)
                if tof:
                    try:
                        tof.start_ranging(3)
                    except Exception:
                        pass

            while True:
                if timeout is not None and (time.time() - start_time) > timeout:
                    return {
                        "baseline_roi_mean": baseline_mean,
                        "current_roi_mean": None,
                        "diff": None,
                        "is_cliff": False,
                        "checks": checks,
                        "consecutive_required": consecutive,
                        "reason": "timeout",
                    }

                # Collect a short ROI sample set for this check
                roi_samples = self._sample_roi_via_tof(
                    which, check_samples, check_delay, roi_preset
                )
                checks += 1

                if not roi_samples:
                    self.logger.debug("No ROI samples collected this check; skipping")
                    time.sleep(interval)
                    continue

                current_vals = [r for r in roi_samples if r is not None]
                current_mean = mean(current_vals) if current_vals else None

                if current_mean is None:
                    self.logger.debug("ROI samples all None this check; skipping")
                    time.sleep(interval)
                    continue

                diff = abs(current_mean - baseline_mean)
                self.logger.info(
                    f"Check {checks}: roi_mean={current_mean} mm, diff={diff} mm"
                )

                if diff >= threshold_mm:
                    consec += 1
                    self.logger.debug(
                        f"Consecutive threshold breaches: {consec}/{consecutive}"
                    )
                    if consec >= consecutive:
                        return {
                            "baseline_roi_mean": baseline_mean,
                            "current_roi_mean": current_mean,
                            "diff": diff,
                            "is_cliff": True,
                            "checks": checks,
                            "consecutive_required": consecutive,
                        }
                else:
                    consec = 0

                time.sleep(interval)

        except KeyboardInterrupt:
            return {
                "baseline_roi_mean": baseline_mean,
                "current_roi_mean": None,
                "diff": None,
                "is_cliff": False,
                "checks": checks,
                "consecutive_required": consecutive,
                "reason": "interrupted",
            }


def main():
    log = logging.getLogger("cliff_test")
    logging.basicConfig(level=logging.INFO)

    tester = CliffTester(logger=log)
    baseline = tester.establish_roi_baseline(
        which="rear", roi_preset="top", baseline_samples=20, sample_delay=0.05
    )
    if not baseline:
        print("Failed to establish baseline; can't run monitor.")
        return

    out = tester.monitor_for_cliff(
        which="rear",
        roi_preset="top",
        baseline=baseline,
        check_samples=3,
        check_delay=0.05,
        interval=1.0,
        threshold_mm=200,
        consecutive=3,
        timeout=30,
    )
    print("Cliff monitor output:", out)


if __name__ == "__main__":
    main()
