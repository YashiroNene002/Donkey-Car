import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import hailo
import time
import smbus

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

# I2C setup
I2C_BUS = 1  # I2C bus (usually 1 on Raspberry Pi)
I2C_ADDRESS = 0x08  # I2C address of the slave device
bus = smbus.SMBus(I2C_BUS)

# ฟังก์ชันส่งข้อมูลสถานะผ่าน I2C
def update_status(detected_count):
    try:
        # ส่งจำนวนรถที่ตรวจพบ (1 byte for count)
        bus.write_byte(I2C_ADDRESS, detected_count)
    except Exception as e:
        print(f"Failed to send I2C message: {e}")

class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()

# Callback Function
def app_callback(pad, info, user_data):
    
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()

    # ตรวจจับโมเดล
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    car_detected_count = 0  # จำนวนรถที่ตรวจพบ
    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()
        if label == "car" and confidence > 0.5:
            car_detected_count += 1

    # ส่งจำนวนรถที่ตรวจพบผ่าน I2C
    update_status(car_detected_count)

    print(f"Cars detected: {car_detected_count}")

    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
