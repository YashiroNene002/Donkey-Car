import tflite_runtime.interpreter as tflite
import cv2
import numpy as np
import time
import paho.mqtt.client as mqtt
from YB_Pcb_Car import YB_Pcb_Car
import RPi.GPIO as GPIO

# กำหนดพอร์ตสำหรับ HC-SR04
TRIG = 23
ECHO = 24

# กำหนด GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# โหลดโมเดล MobileNet
model_path = "/home/pi/Downloads/mobilenetv2_lane_detection_tuneroom1.tflite"
try:
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    print("Model loaded successfully")
except Exception as e:
    print(f"Error loading model: {e}")
    exit(1)

IMG_SIZE = 224
TARGET_FPS = 25
FRAME_DURATION = 1 / TARGET_FPS

def preprocess_image(frame):
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = img / 255.0
    img = img.astype(np.float32)
    return np.expand_dims(img, axis=0)

# ฟังก์ชันควบคุมรถ
def control_car(steering, left_speed, right_speed, car):
    if steering is None or left_speed is None or right_speed is None:
        car.Car_Stop()
        return

    steering = np.clip(steering, -1.0, 1.0)
    left_speed = np.clip(left_speed, 0, 255)
    right_speed = np.clip(right_speed, 0, 255)

    car.Ctrl_Car(1, int(left_speed), 1, int(right_speed))

def distance_measurement():
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)
    pulse_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    pulse_end = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

broker = "localhost"
port = 1883
topic = "traffic/status"
person_detected_count = 0
traffic_light_status = None

def on_message(client, userdata, msg):
    global person_detected_count, traffic_light_status
    try:
        status = eval(msg.payload.decode())
        person_detected_count = status.get("person_detected_count", 0)
        traffic_light_status = status.get("traffic_light_status", None)
    except Exception as e:
        print(f"Error parsing message: {e}")

client = mqtt.Client()
client.on_message = on_message
client.connect(broker, port, 60)
client.subscribe(topic)
client.loop_start()

cap = cv2.VideoCapture(0)
car = YB_Pcb_Car()
prev_time = time.time()

try:
    while True:
        frame_start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        input_image = preprocess_image(frame)
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        interpreter.set_tensor(input_details[0]['index'], input_image.astype(np.float32))
        interpreter.invoke()
        predictions = interpreter.get_tensor(output_details[0]['index'])[0]

        steering, left_speed, right_speed = predictions

        if person_detected_count > 0:
            distance = distance_measurement()
            if distance <= 50:
                control_car(None, 0, 0, car)
                print("🚨 Person detected within 50 cm, stopping...")
                continue

        if traffic_light_status == "Green":
            control_car(steering, left_speed, right_speed, car)
            print("✅ No person detected and green traffic light. Proceeding...")
        elif traffic_light_status == "Yellow":
            speed = 40  # กำหนดความเร็วเริ่มต้น
            while speed > 0:
                control_car(steering, speed, speed, car)
                print(f"⚠️ No person detected and yellow traffic light. Slowing down... Speed: {speed}")
                speed -= 10  # ลดความเร็วลงทีละ 5 (ปรับค่าได้ตามต้องการ)
                time.sleep(0.75)  # หน่วงเวลาเพื่อให้การลดความเร็วเป็นธรรมชาติ
            control_car(steering, 0, 0, car)  # หยุดรถเมื่อความเร็วเหลือ 0
            print("🚦 Car stopped due to yellow light.")

        elif traffic_light_status == "Red": 
            control_car(steering, 0, 0, car)
            print("🚦 No person detected and red traffic light. Stopping...")
        else:
            # หากไม่มีข้อมูลไฟจราจรและไม่พบคน ให้รัน MobileNet
            control_car(steering, left_speed, right_speed, car)
            print("🔄 No person or traffic light detected. Running MobileNet model...")

        current_time = time.time()
        fps = 1 / (current_time - prev_time)
        prev_time = current_time
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("Camera", frame)

        elapsed_time = time.time() - frame_start_time
        sleep_time = max(0, FRAME_DURATION - elapsed_time)
        time.sleep(sleep_time)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    car.Car_Stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
