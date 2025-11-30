import logging
import time
from statistics import mean

import VL53L1X
from tof_sensor import ToFSensor


class CliffTester(ToFSensor):
    """
    Subclass of ToFSensor that runs a simple cliff-detection routine by
    comparing the normal center reading to a ROI-biased reading (e.g. top row).
    """

    def detect_cliff(
        self,
        which="rear",
        samples=15,
        delay=0.05,
        roi_preset="top",
        threshold_mm=200,
    ):
        """
        Compare center measurement to ROI-biased (edge) measurements.

        Returns:
          dict with:
            - 'primary': center measurement (or None)
            - 'roi_samples': list of ROI measurements (may be empty)
            - 'roi_mean': average of roi_samples or None
            - 'diff': absolute difference between primary and roi_mean (or None)
            - 'is_cliff': True/False/None
        """
        try:
            self.enable(which)
            time.sleep(self.settle)  # short settle so ranging stabilizes
        except Exception:
            # If enable isn't available for some reason, try starting ranging directly
            tof = self._tofs.get(which)
            if tof:
                try:
                    tof.start_ranging(3)
                    time.sleep(self.settle)
                except Exception:
                    pass
        result = {
            "primary": None,
            "roi_samples": [],
            "roi_mean": None,
            "diff": None,
            "is_cliff": None,
        }

        # Primary (center) reading
        primary = self.get_distance(which)
        result["primary"] = primary

        # If a user-provided cliff_detection exists on the base class and it
        # returns samples, use it. Otherwise we'll do our own ROI sampling.
        if hasattr(self, "cliff_detection") and callable(
            getattr(self, "cliff_detection")
        ):
            # Avoid infinite recursion: only use base implementation if it's defined
            # on the parent class (ToFSensor), not this subclass. Check the MRO.
            base_method = None
            for cls in type(self).mro()[1:]:
                if "cliff_detection" in cls.__dict__:
                    base_method = cls.__dict__.get("cliff_detection")
                    break

            if base_method:
                try:
                    maybe = base_method(self, which=which, samples=samples, delay=delay)
                    if isinstance(maybe, (list, tuple)) and maybe:
                        result["roi_samples"] = list(maybe)
                except TypeError:
                    # signature mismatch — fall back
                    pass
                except Exception:
                    # base method might only print — ignore and fall back
                    pass

        # If no roi_samples obtained above, do an explicit ROI sampling using the low-level VL53L1X object.
        if not result["roi_samples"]:
            tof = self._tofs.get(which)
            if tof is None:
                self.logger.error(f"Sensor '{which}' not found for ROI sampling")
            else:
                # Map friendly preset names to ROI args used in your wrapper (as in your original code)
                presets = {
                    "top": (0, 15, 15, 15),
                    "middle": (7, 15, 15, 15),
                    "bottom": (15, 15, 15, 15),
                }
                roi_args = presets.get(roi_preset)
                if roi_args is None:
                    self.logger.error(
                        f"Unknown roi_preset '{roi_preset}'; valid keys: {list(presets.keys())}"
                    )
                else:
                    set_roi_success = False
                    # Try available approaches in order:
                    # 1) Library-level ROI class (VL53L1xUserRoi)
                    # 2) Instance method set_user_roi with args
                    # 3) Instance method set_user_roi with tuple/list
                    # 4) Other plausible method names
                    try:
                        if hasattr(VL53L1X, "VL53L1xUserRoi"):
                            try:
                                roi_obj = VL53L1X.VL53L1xUserRoi(*roi_args)
                                try:
                                    tof.stop_ranging()
                                except Exception:
                                    pass
                                tof.set_user_roi(roi_obj)
                                set_roi_success = True
                            except Exception as e:
                                self.logger.debug(
                                    f"ROI class present but set_user_roi(ROI) failed: {e}"
                                )
                    except Exception:
                        # safe-guard: some wrappers may raise on attribute check
                        pass

                    # Try calling set_user_roi with 4 ints
                    if not set_roi_success and hasattr(tof, "set_user_roi"):
                        try:
                            try:
                                tof.stop_ranging()
                            except Exception:
                                pass
                            tof.set_user_roi(*roi_args)
                            set_roi_success = True
                        except Exception as e:
                            self.logger.debug(f"set_user_roi(*args) failed: {e}")
                            # Try passing tuple
                            try:
                                tof.set_user_roi(tuple(roi_args))
                                set_roi_success = True
                            except Exception as e2:
                                self.logger.debug(f"set_user_roi(tuple) failed: {e2}")

                    # Try alternate method names that some wrappers use
                    alt_names = (
                        "set_roi",
                        "setUserRoi",
                        "user_roi",
                        "set_user_roi_area",
                    )
                    if not set_roi_success:
                        for name in alt_names:
                            if hasattr(tof, name):
                                try:
                                    try:
                                        tof.stop_ranging()
                                    except Exception:
                                        pass
                                    getattr(tof, name)(*roi_args)
                                    set_roi_success = True
                                    break
                                except Exception as e:
                                    self.logger.debug(f"{name}(*args) failed: {e}")

                    if not set_roi_success:
                        self.logger.error(
                            "Unable to set ROI: wrapper doesn't expose a known ROI API. "
                            "Inspect the VL53L1X wrapper on your Pi (dir(VL53L1X) and dir(tof)) and tell me the available methods."
                        )
                    else:
                        # If ROI was (probably) applied, start ranging and sample
                        try:
                            tof.start_ranging(3)
                        except Exception:
                            # continue anyway; some wrappers don't need/rely on start_ranging here
                            pass

                        samples_collected = []
                        for _ in range(samples):
                            try:
                                d = tof.get_distance()
                                samples_collected.append(d)
                            except Exception:
                                samples_collected.append(None)
                            time.sleep(delay)

                        # Filter out None and zeros (if your wrapper returns 0 for out-of-range)
                        filtered = [s for s in samples_collected if s not in (None, 0)]
                        result["roi_samples"] = (
                            filtered if filtered else samples_collected
                        )

                        # Try to leave the sensor running
                        try:
                            tof.start_ranging(3)
                        except Exception:
                            pass

        # Compute statistics and decide
        roi_samples = result["roi_samples"]
        if roi_samples:
            try:
                roi_mean = mean([s for s in roi_samples if s is not None])
                result["roi_mean"] = roi_mean
            except Exception:
                result["roi_mean"] = None

        if result["primary"] is not None and result["roi_mean"] is not None:
            diff = abs(result["primary"] - result["roi_mean"])
            result["diff"] = diff
            result["is_cliff"] = diff >= threshold_mm
        else:
            result["is_cliff"] = None

        return result


def main():
    # Simple demo run. Make sure this script runs on the robot where GPIO/I2C are available.
    log = logging.getLogger("cliff_test")
    logging.basicConfig(level=logging.INFO)

    tester = CliffTester(logger=log)
    # Test rear sensor for a possible cliff
    out = tester.detect_cliff(
        which="rear", samples=5, delay=0.05, roi_preset="top", threshold_mm=200
    )
    print("Cliff test output:", out)


if __name__ == "__main__":
    main()
