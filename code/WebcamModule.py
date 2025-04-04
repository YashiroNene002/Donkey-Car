import cv2
import sys
import os

os.environ["SDL_VIDEODRIVER"] = "dummy"

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    sys.exit()

def getImg(display=False, size=[480, 240]):
    ret, img = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        return None
    img = cv2.resize(img, (size[0], size[1]))
    if display:
        cv2.imshow('IMG', img)
    return img

if __name__ == '__main__':
    while True:
        img = getImg(True)
        if img is None:
            break
    cap.release()
    cv2.destroyAllWindows()
