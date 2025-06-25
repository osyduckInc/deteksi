import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time

cap = cv2.VideoCapture(0)
ws, hs = 1920, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

port = "/dev/ttyACM1"
board = pyfirmata.Arduino(port)
servo_pinX = board.get_pin('d:9:s') #pin 9 Arduino
servo_pinY = board.get_pin('d:10:s') #pin 10 Arduino
led_pin = board.get_pin('d:5:o') # Pin 5 Arduino

detector = FaceDetector()
servoPos = [90, 90] # initial servo position

# Variable to flip Y mapping
flip_y_mapping = False  # Press 'f' to toggle this

# Initialize servo to center position
servo_pinX.write(servoPos[0])
servo_pinY.write(servoPos[1])
time.sleep(1)

print("Face tracking started...")

while True:
    success, img = cap.read()
    
    img = cv2.flip(img, 1)
    
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Face detected - LED on
        led_pin.write(1)
        
        # Get the coordinate
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        pos = [fx, fy]
        
        # Convert coordinate to servo degree
        # With horizontal flip, use normal mapping (not inverted)
        servoX = np.interp(fx, [0, ws], [0, 180])  # Normal mapping for flipped image
        
        # Y mapping - toggle with 'f' key if direction is wrong
        if flip_y_mapping:
            servoY = np.interp(fy, [0, hs], [180, 0])  # Inverted Y mapping
        else:
            servoY = np.interp(fy, [0, hs], [0, 180])  # Normal Y mapping
        
        # Constrain servo values
        servoX = max(0, min(180, servoX))
        servoY = max(0, min(180, servoY))

        # Update servo positions only when face is detected
        servoPos[0] = servoX
        servoPos[1] = servoY
        
        # Write to servos
        servo_pinX.write(servoPos[0])
        servo_pinY.write(servoPos[1])

        # Visual feedback
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    else:
        # No face detected - LED off, servo stays in last position
        led_pin.write(0)
        
        # Visual feedback - show waiting state
        cv2.putText(img, "WAITING...", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3)
        cv2.circle(img, (ws//2, hs//2), 80, (0, 255, 0), 2)
        cv2.circle(img, (ws//2, hs//2), 15, (0, 255, 0), cv2.FILLED)
        cv2.line(img, (0, hs//2), (ws, hs//2), (0, 0, 0), 2)  # x line
        cv2.line(img, (ws//2, hs), (ws//2, 0), (0, 0, 0), 2)  # y line
        
        # Servo tetap di posisi terakhir (tidak bergerak)

    cv2.imshow("Image", img)
    
    # Keyboard controls
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):  # Reset servo to center position
        servoPos = [90, 90]
        servo_pinX.write(servoPos[0])
        servo_pinY.write(servoPos[1])
        print("Servo reset to center position")
    elif key == ord('f'):  # Toggle Y mapping
        flip_y_mapping = not flip_y_mapping
        print(f"Y mapping flipped: {flip_y_mapping}")

# Cleanup
cap.release()
cv2.destroyAllWindows()
board.exit()
