import logging
import pprint
import time

import VL53L1X
from tof_sensor import ToFSensor

print("Module VL53L1X attrs:")
pprint.pprint(sorted([a for a in dir(VL53L1X) if not a.startswith("_")]))

log = logging.getLogger("inspect")
logging.basicConfig(level=logging.INFO)
t = ToFSensor(logger=log)
tof = t._tofs.get("rear") or t._tofs.get("front")
print("\nInstance `tof` type:", type(tof))
print("tof attrs:")
pprint.pprint(sorted([a for a in dir(tof) if not a.startswith("_")]))
