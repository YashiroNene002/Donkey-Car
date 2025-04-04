import WebcamModule as wM
import DataCollectionModule as dcM
import JoyStickModule as jsM
import YB_Pcb_Car
from time import sleep

# Motor and joystick configuration
car = YB_Pcb_Car.YB_Pcb_Car()
maxThrottle = 255  # Maximum speed
throttle_sensitivity = 0.5
steering_sensitivity = 0.3  # Reduced steering sensitivity for better control
record = False  # Start in recording mode off

while True:
    joyVal = jsM.getJS()  # Get joystick values
    steering = joyVal['axis3']  # Steering control
    throttle = -joyVal['axis2'] * maxThrottle * throttle_sensitivity  # Inverted throttle for correct direction

    # Toggle recording with 'share' button
    if joyVal['share'] == 1:
        record = not record
        print('Recording Started' if record else 'Recording Stopped')

    # Set the car's speed and direction based on joystick input
    left_speed = int(throttle + (steering * maxThrottle * steering_sensitivity))
    right_speed = int(throttle - (steering * maxThrottle * steering_sensitivity))

    car.Control_Car(left_speed, right_speed)  # Control the car with calculated speeds

    # Log data if throttle is non-zero (indicating motors are active)
    if throttle != 0:
        img = wM.getImg(False, size=[240, 120])
        if img is not None:
            dcM.saveData(img, steering, left_speed, right_speed)  # Save image, steering angle, and motor speeds

    # Save log file when 'window' button is pressed
    if joyVal['window'] == 1:
        dcM.saveLog()
        record = False
        print("Log Saved")

    sleep(0.05)
