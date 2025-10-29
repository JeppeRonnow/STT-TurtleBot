import time
import sys
import signal

I2C_BUS = 1
I2C_BUS = 2
I2C_ADDR = 0x29
GRID_SIZE = 16                # 16x16 scan (consider 8 for speed)
THRESH_MM = 300               # 300 mm threshold
INTER_MEASUREMENT_MS = 20     # Sensor timing budget interplay; tune as needed
SLEEP_BETWEEN_ZONES = 0.0

import VL53L1X

import threading as thread

class ToF_sensor:
    # Variables
    STOP_THRESHOLD = 0.5  # distance in meters to trigger stop

    # add custom struct: 16x16 matrix variable. The sensor data will be stored in this matrix.

    def __init__(self):
        # initialize the sensor first then start a new thread for detecting objects

        self.floor_distance = self.get_floor_distance()
        thread.Thread(target = self.object_detection).start()

    def make_roi(top, left, bottom, right)

    def get_floor_distance(self) -> float:
        # return the distance to the floor from the sensor once on initialization
        return 0.0


    def get_data(self) -> matrix<16,16>:
        # return the current sensor data as a 16x16 matrix


    def stop_robot(self):
        print("Object detected! Stopping the robot.")


    def object_detection(self):
        # continuously monitor the sensor data for object detection
        while True:
            data = self.get_data()
            for value in data: #only check top 4 rows
                if value < self.STOP_THRESHOLD:
                    self.stop_robot()
            for value in data: #check floor distance
                if value < self.floor_distance - 0.1: #if an object is detected closer than the floor distance minus a small margin
                    self.stop_robot()