#!usr/bin/env python3

import cv2
import time
from display import Display

W = 1920 // 2
H = 1080 // 2

# display object
disp = Display("Display Window", W, H)

def process_frame(img):
    img = cv2.resize(img, (W, H))
    # cv2.imshow('image', img)
    disp.paint(img)
    print(img)

def main():
    video = cv2.VideoCapture("video 1.mp4")
    if video.isOpened() == False:
        print("Error, video file could not be loaded")

    while (video.isOpened()):
        ret, frame = video.read()
        if ret == True:
            process_frame(frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        else:
            break

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
