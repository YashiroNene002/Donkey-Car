import tflite_runtime.interpreter as tflite
import cv2
import numpy as np
from YB_Pcb_Car import YB_Pcb_Car  # นำเข้าโมดูลควบคุมรถ

# โหลดโมเดล TensorFlow Lite
model_path = '/home/pi/Downloads/final_model1.tflite'  # เปลี่ยนเป็นไฟล์ .tflite
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# ขนาดภาพที่ใช้กับโมเดล
IMG_SIZE = 224

# ฟังก์ชันสำหรับประมวลผลภาพ
def preprocess_image(frame):
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))  # ปรับขนาดภาพให้ตรงกับโมเดล
    img = img / 255.0  # ปรับภาพให้เป็น [0, 1] (ถ้าจำเป็น)
    return np.expand_dims(img, axis=0)  # เพิ่ม batch dimension

# ฟังก์ชันควบคุมรถ
def control_car(steering, left_speed, right_speed, car):
    """
    แปลงค่าจากโมเดลเป็นคำสั่งควบคุม Yahboom Car
    """
    try:
        # กำหนดทิศทางและความเร็วตาม output ของโมเดล
        left_direction = 1 if left_speed >= 0 else 0
        right_direction = 1 if right_speed >= 0 else 0

        # ส่งคำสั่งไปยังรถ
        car.Ctrl_Car(
            left_direction, int(abs(left_speed)), 
            right_direction, int(abs(right_speed))
        )
    except Exception as e:
        print(f"Error in control_car: {e}")

# เริ่มต้นกล้องและโมดูลควบคุมรถ
cap = cv2.VideoCapture(0)  # เปิดกล้องที่เชื่อมต่อกับ Raspberry Pi
car = YB_Pcb_Car()  # สร้างอินสแตนซ์ของรถ

try:
    while True:
        # อ่านภาพจากกล้อง
        ret, frame = cap.read()
        if not ret:
            print("ไม่สามารถอ่านภาพจากกล้องได้")
            break

        # ประมวลผลภาพ
        input_image = preprocess_image(frame)

        # เตรียมข้อมูลให้เหมาะสมกับโมเดล
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        interpreter.set_tensor(input_details[0]['index'], input_image)
        interpreter.invoke()

        # ดึงผลลัพธ์จากโมเดล
        predictions = interpreter.get_tensor(output_details[0]['index'])[0]
        steering, left_speed, right_speed = predictions  # ดึงค่าจาก output

        # ควบคุมรถด้วยค่าที่พยากรณ์ได้
        control_car(steering, left_speed, right_speed, car)

        # แสดงภาพจากกล้อง (สำหรับ debug)
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # กด 'q' เพื่อหยุดการทำงาน
            break
finally:
    # ปิดกล้องและหยุดรถ
    cap.release()
    car.Car_Stop()  # หยุดรถ
    cv2.destroyAllWindows()
