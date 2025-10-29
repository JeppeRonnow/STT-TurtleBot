import threading as thread
import RPi.GPIO as GPIO
import time

class Matrix:
    def __init__(self):
        self.matrix = [[0.0 for _ in range(16)] for _ in range(16)]
        print("Custom struct 'dataStruct' initialized.")
        print(self.matrix)

class TOF_Sensor:
    # Variables
    STOP_THRESHOLD = 0.5  # distance in meters to trigger stop
    LED_PIN = 12  # BCM pin 18 (physical pin 12)

    def __init__(self):
        # initialize the sensor first then start a new thread for detecting objects

        self.matrix = Matrix()

        GPIO.setmode(GPIO.BOARD)    # or GPIO.BOARD for physical numbering
        GPIO.setup(self.LED_PIN, GPIO.OUT)

        self.blink_led()

        self.floor_distance = self.get_floor_distance()
        thread.Thread(target = self.object_detection).start()
    
    def get_floor_distance(self) -> float:
        # return the distance to the floor from the sensor once on initialization
        data = self.get_data()
        return data[15][8]  # assuming the floor distance is at the center of the last row

    def get_data(self) -> Matrix:
        # return the current sensor data as a 16x16 matrix
        pass

    def stop_robot(self) -> None:
        print("Object detected! Stopping the robot...")
        # publish a zero velocity command to stop the robot

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
        while True:
            data = self.get_data()
            for value in data: #only check top 4 rows
                if value < self.STOP_THRESHOLD:
                    self.stop_robot()
            for value in data: #check floor distance
                if value < self.floor_distance - 0.1: #if an object is detected closer than the floor distance minus a small margin
                    self.stop_robot()

if __name__ == "__main__":
    print("Initializing TOF Sensor...")
    sensor = TOF_Sensor()
    print("TOF Sensor initialized and running.")