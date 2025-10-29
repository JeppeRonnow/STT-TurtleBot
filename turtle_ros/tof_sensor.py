import VL53L1X
import signal
import time
import sys
import threading as thread
import RPi.GPIO as GPIO
import time

class Matrix:
    def __init__(self):
        self.matrix = [[0.0 for _ in range(16)] for _ in range(16)]
        print("Custom struct 'Matrix' initialized.")
    
    def __list__(self, row, col):
        return self.matrix[row][col]
    
    def __str__(self):
        print("\nMatrix Data:")
        for row in self.matrix:
            for col in row:
                print(f"{col:.2f}", end=" ")
        print("\n")

class TOF_Sensor:
    # Variables
    I2C_BUS = 1
    I2C_ADDR = 0x29
    GRID_SIZE = 16                # 16x16 scan (consider 8 for speed)
    THRESH_MM = 300               # 300 mm threshold
    INTER_MEASUREMENT_MS = 20     # Sensor timing budget interplay; tune as needed
    SLEEP_BETWEEN_ZONES = 0.0     # Small delay after set_user_roi (0–5 ms typically
    STOP_THRESHOLD = 0.5  # distance in meters to trigger stop
    LED_PIN = 18  # GPIO/BCM pin 18 (physical pin 12)
   

    def __init__(self):
        print("Initializing TOF Sensor...")
        # initialize the sensor first then start a new thread for detecting objects

        self.matrix = Matrix()

        GPIO.setmode(GPIO.BCM)    # or GPIO.BOARD for physical numbering
        GPIO.setup(self.LED_PIN, GPIO.OUT)

        self.blink_led()

        self.floor_distance = self.get_floor_distance()
        thread.Thread(target = self.object_detection).start()

        print("[TOF Sensor initialized]")


    def make_roi(top, left, bottom, right):
        pass


    def get_floor_distance(self) -> float:
        # return the distance to the floor from the sensor once on initialization
        data = self.get_data()
        return data(15,8)  # assuming the floor distance is at the center of the last row


    def get_data(self) -> Matrix:
        return self.matrix # Change this to return actual sensor data


    def stop_robot(self) -> None:
        print("Object detected! Stopping the robot...")
        # publish a zero velocity command to stop the robot


    def blink_led(self) -> None:
        for _ in range(3):  # Blink 3 times on initialization
            print("Blinking LED...")
            GPIO.output(self.LED_PIN, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(self.LED_PIN, GPIO.LOW)
            time.sleep(0.5)


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


if __name__ == "__main__":
    try:
        TOF_Sensor()
        print("TOF Sensor initialized and running.")
    except KeyboardInterrupt:
        print("Shutting down TOF Sensor...")
    finally:
        GPIO.cleanup()