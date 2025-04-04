import pandas as pd
import os
import cv2
from datetime import datetime

global imgList, steeringList, leftSpeedList, rightSpeedList, imgCounter
countFolder = 0
imgList = []
steeringList = []
leftSpeedList = []
rightSpeedList = []
imgCounter = 0  # Image counter for ordered naming

# GET CURRENT DIRECTORY PATH
myDirectory = os.path.join(os.getcwd(), 'DataCollected')

# CREATE A NEW FOLDER BASED ON THE PREVIOUS FOLDER COUNT
while os.path.exists(os.path.join(myDirectory, f'IMG{str(countFolder)}')):
    countFolder += 1
newPath = os.path.join(myDirectory, f"IMG{countFolder}")
os.makedirs(newPath, exist_ok=True)

# SAVE IMAGES IN THE FOLDER
def saveData(img, steering, left_speed, right_speed):
    global imgList, steeringList, leftSpeedList, rightSpeedList, imgCounter
    if img is None:
        print("Error: Image capture failed, not saving data.")
        return
    # Create ordered file name based on imgCounter
    fileName = os.path.join(newPath, f'Image_{imgCounter:04d}.jpg')
    cv2.imwrite(fileName, img)
    imgList.append(fileName)
    steeringList.append(steering)
    leftSpeedList.append(left_speed)
    rightSpeedList.append(right_speed)
    imgCounter += 1  # Increment the image counter for the next image

# SAVE LOG FILE WHEN THE SESSION ENDS
def saveLog():
    global imgList, steeringList, leftSpeedList, rightSpeedList
    # ดึงแค่ชื่อไฟล์ของรูปภาพ (ไม่รวม path)
    imgFilenames = [os.path.basename(img) for img in imgList]
    
    rawData = {
        'Image': imgFilenames,  # เก็บแค่ชื่อไฟล์ของรูปภาพ
        'Steering': steeringList, 
        'Left_Speed': leftSpeedList, 
        'Right_Speed': rightSpeedList
    }
    
    df = pd.DataFrame(rawData)
    logFilePath = os.path.join(myDirectory, f'log_{countFolder}.csv')
    df.to_csv(logFilePath, index=False, header=True)
    print(f'Log saved: {logFilePath}')
    print('Total Images:', len(imgList))


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Camera could not be opened.")
        sys.exit()

    while True:  # ลูปนี้จะทำงานตลอดไปจนกว่าผู้ใช้จะหยุด
        ret, img = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break
        
        saveData(img, 0.5, 255, 255)  # บันทึกรูปและข้อมูล (ตัวอย่าง dummy values)
        cv2.imshow("Image", img)
        
        # กด 'q' เพื่อหยุดการทำงาน
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    saveLog()  # บันทึก log เมื่อการทำงานเสร็จสิ้น
    cap.release()
    cv2.destroyAllWindows()
