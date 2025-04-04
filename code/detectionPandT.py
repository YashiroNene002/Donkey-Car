import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
import time
import paho.mqtt.client as mqtt
from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

# MQTT Setup
broker = "localhost"  # หรือ "127.0.0.1"
port = 1883
topic = "traffic/status"

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")

client = mqtt.Client()
client.on_connect = on_connect
client.connect(broker, port, 60)
client.loop_start()

# ฟังก์ชันส่งข้อมูลสถานะผ่าน MQTT
def update_status(person_detected_count, traffic_light_status):
    status = {
        "person_detected_count": person_detected_count,  # ส่งจำนวนคนที่ตรวจพบ
        "traffic_light_status": traffic_light_status  # ส่งสถานะไฟจราจร
    }
    client.publish(topic, str(status))

# เพิ่มการกำหนดค่าให้กับตัวแปร
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.person_detected_count = 0  # กำหนดค่าเริ่มต้นให้กับ person_detected_count
        self.traffic_light_status = None  # กำหนดค่าเริ่มต้นให้กับ traffic_light_status

# Callback Function
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()

    # ตรวจจับโมเดล
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    person_detected_count = 0  # จำนวนคนที่ตรวจพบ
    traffic_light_status = None  # สถานะไฟจราจร (แดง, เหลือง, เขียว)

    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()

        # ตรวจจับคน
        if label == "person" and confidence > 0.5:
            person_detected_count += 1

        # ตรวจจับไฟจราจร
        elif label.startswith("traffic light") and confidence > 0.1:
            if label == "traffic light red":
                traffic_light_status = "Red"
            elif label == "traffic light yellow":
                traffic_light_status = "Yellow"
            elif label == "traffic light green":
                traffic_light_status = "Green"

    # ส่งจำนวนคนและสถานะไฟจราจรผ่าน MQTT
    update_status(person_detected_count, traffic_light_status)

    # บันทึกค่าที่ตรวจจับได้ใน user_data เพื่อให้สามารถใช้ในฟังก์ชันหลัก
    user_data.person_detected_count = person_detected_count
    user_data.traffic_light_status = traffic_light_status

    # แสดงผลที่หน้าจอ
    print(f"People detected: {person_detected_count}")
    print(f"Traffic light status: {traffic_light_status}")

    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
