import VL53L1X
import signal
import time
import sys
import RPi.GPIO as GPIO
import time

class Matrix:
    def __init__(self):
        self.matrix = [[0.0 for _ in range(16)] for _ in range(16)]
        print("Custom struct 'dataStruct' initialized.")
        print(self.matrix)

class TOF_Sensor:
    # Variables
    I2C_BUS = 1
    I2C_ADDR = 0x29
    GRID_SIZE = 16                # 16x16 scan (consider 8 for speed)
    THRESH_MM = 300               # 300 mm threshold
    INTER_MEASUREMENT_MS = 20     # Sensor timing budget interplay; tune as needed
    SLEEP_BETWEEN_ZONES = 0.0     # Small delay after set_user_roi (0–5 ms typically
    STOP_THRESHOLD = 0.5          # distance in meters to trigger stop
    LED_PIN = 18                  # BCM pin 18 (physical pin 12)

    # add custom struct: 16x16 matrix variable. The sensor data will be stored in this matrix.
    def make_roi(top, left, bottom, right):
        pass
        

    def __init__(self, logger):
        self.logger = logger

        # initialize the sensor
        self.matrix = Matrix()
        GPIO.setmode(GPIO.BOARD)    # or GPIO.BOARD for physical numbering
        GPIO.setup(self.LED_PIN, GPIO.OUT)

        self.blink_led()

        self.floor_distance = self.get_floor_distance()
 

    def get_floor_distance(self) -> float:
        # return the distance to the floor from the sensor once on initialization
        data = self.get_data()
        return data[15][8]  # assuming the floor distance is at the center of the last row


    def get_data(self) -> Matrix:
        # return the current sensor data as a 16x16 matrix
        pass


    def blink_led(self):
        try:
            while True:
                GPIO.output(self.LED_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(self.LED_PIN, GPIO.LOW)
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            GPIO.cleanup()


    def object_detection(self):
        # continuously monitor the sensor data for object detection
        data = self.get_data()
        for value in data: # Only check top 4 rows
            if value < self.STOP_THRESHOLD:
                return True
        for value in data: #check floor distance
            if value < self.floor_distance - 0.1: #if an object is detected closer than the floor distance minus a small margin
                return True

        return False


if __name__ == "__main__":
    print("Initializing TOF Sensor...")
    sensor = TOF_Sensor()
    print("TOF Sensor initialized and running.")
