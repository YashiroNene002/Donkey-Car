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

# ตรวจสอบการโหลดโมเดล
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

# ฟังก์ชันประมวลผลภาพ
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

# ฟังก์ชันสำหรับวัดระยะห่าง
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
    distance = pulse_duration * 17150  # ความเร็วเสียงในอากาศ
    distance = round(distance, 2)
    
    return distance

# MQTT Setup
broker = "localhost"  # หรือ "127.0.0.1"
port = 1883
topic = "car/status"

car_detected_count = 0  # เริ่มต้นค่า car_detected_count เป็น 0

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")

def on_message(client, userdata, msg):
    global car_detected_count
    try:
        # รับข้อมูลจาก MQTT และแปลงเป็น dictionary
        status = eval(msg.payload.decode())
        car_detected_count = status.get("car_detected_count", 0)  # รับจำนวนรถที่ตรวจพบ
    except Exception as e:
        print(f"Error parsing message: {e}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker, port, 60)
client.subscribe(topic)
client.loop_start()

# เริ่มต้นกล้อง
cap = cv2.VideoCapture(0)
car = YB_Pcb_Car()
prev_time = time.time()
state = "normal"  # สถานะการขับ (normal, slow, stop, overtake)

distance_check_interval = 5  # ตรวจสอบระยะห่างทุกๆ 5 เฟรม
last_distance_check_time = time.time()

# ปรับการทำงานให้ดีขึ้น
try:
    while True:
        frame_start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        input_image = preprocess_image(frame)

        # ใช้โมเดล MobileNet สำหรับขับเคลื่อน
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        interpreter.set_tensor(input_details[0]['index'], input_image.astype(np.float32))
        interpreter.invoke()
        predictions = interpreter.get_tensor(output_details[0]['index'])[0]

        steering, left_speed, right_speed = predictions

        # ถ้ายังไม่เจอรถ
        if car_detected_count == 0:
            control_car(steering, left_speed, right_speed, car)  # รถวิ่งต่อไปตามปกติ
            print("🚗 No car detected, continuing to drive...")
        else:
            # ตรวจสอบระยะห่างทุกๆ 5 เฟรม
            if time.time() - last_distance_check_time > distance_check_interval * FRAME_DURATION:
                distance = distance_measurement()
                last_distance_check_time = time.time()
                print(f"Cars detected: {car_detected_count}")
                print(f"🚗 Car detected! Distance: {distance} cm")
                
                # ถ้าระยะห่างน้อยกว่า 100 ซม. ลดความเร็ว
                if distance <= 100:
                    #steering = 1.0
                    #left_speed = 50
                    #right_speed = 50
                    left_speed = max(0, left_speed - 35)
                    right_speed = max(0, right_speed - 35)
                    print("🚨 Car is within 1 meter, slowing down...")
                
                # ถ้าระยะห่างน้อยกว่า 50 ซม. ลดความเร็วลงอีก 50
                if distance <= 50:
                    #steering = 1.0
                    #left_speed = 40
                    #right_speed = 40
                    left_speed = max(0, left_speed - 20)
                    right_speed = max(0, right_speed - 20)
                    print("🚨 Car is within 50 cm, slowing down further...")
                
                
                 # ถ้าระยะห่างน้อยกว่า 25 ซม. และเจอรถ 2 คันหยุด
                if distance <= 50 and car_detected_count >= 2:
                    print("🚨 Two cars detected within 25 cm, stopping until only one car is present...")
                    car.Car_Stop()
                    while car_detected_count >= 2:
                        time.sleep(1)  # รอจนกว่าจะมีรถเหลือแค่ 1 คัน
                        print(f"Cars detected: {car_detected_count}")
                        if car_detected_count == 1:
                            break;
                # ถ้าระยะห่างน้อยกว่า 25 ซม. และเจอรถ 1 คันให้หลบหลีกและแซง
                if distance <= 25:
                    print("🚨 Car is too close, stopping and overtaking...")
                    car.Car_Stop()
                    time.sleep(1)
                    # แซงไปข้างหน้า
                    car.Ctrl_Car(1, 80, 0, 80)  
                    time.sleep(0.65)
                    # แซงซ้าย
                    car.Ctrl_Car(1, 80, 1, 80)
                    time.sleep(0.8)
                    # กลับมาในเลน
                    car.Ctrl_Car(0, 80, 1, 80)
                    time.sleep(0.81)
                    car.Ctrl_Car(1, 200, 1, 200)
                    time.sleep(0.9)
                    car.Ctrl_Car(0, 80, 1, 80)
                    time.sleep(0.3)
                    # กลับมาในเลน
                    car.Ctrl_Car(1, 80, 1, 80)
                    time.sleep(0.8)
                    # กลับมาในเลน
                    car.Ctrl_Car(1, 80, 0, 80)
                    time.sleep(0.7)
                    print("🚨 Overtaking complete, resuming normal driving...")
                else:
                    control_car(steering, left_speed, right_speed, car)  # รถวิ่งต่อไปตามปกติ

        # แสดง FPS
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
