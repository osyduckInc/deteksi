import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time

cap = cv2.VideoCapture(1)
ws, hs = 1920, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

port = "COM12"
board = pyfirmata.Arduino(port)
servo_pinX = board.get_pin('d:9:s') #pin 7 Arduino
servo_pinY = board.get_pin('d:10:s') #pin 10 Arduino
led_pin = board.get_pin('d:5:o') # Pin 5 Arduino

detector = FaceDetector()
servoPos = [90, 90] # initial servo position

# Variables for searching mode
searching = False
search_direction_x = 1  # 1 for right, -1 for left
search_direction_y = 1  # 1 for down, -1 for up
search_step = 5  # degrees per step
search_delay = 0.1  # delay between search steps

# Variables to track search pattern
search_cycles = 0  # Count how many X sweeps completed
max_search_cycles = 3  # Maximum X sweeps before changing Y

# Variable to flip Y mapping
flip_y_mapping = False  # Press 'f' to toggle this

# Initialize servo to center position
servo_pinX.write(servoPos[0])
servo_pinY.write(servoPos[1])
time.sleep(1)

print("Face tracking started...")

while True:
    success, img = cap.read()
    
    # Fix camera orientation - choose one of these options:
    img = cv2.flip(img, 1)  # Horizontal flip (mirror effect) - RECOMMENDED
    # img = cv2.flip(img, 0)  # Vertical flip (upside down fix)
    # img = cv2.flip(img, -1)  # Both horizontal and vertical flip
    
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Face detected - turn off search mode and reset search variables
        searching = False
        search_cycles = 0  # Reset search cycle counter
        led_pin.write(1)
        
        # Get the coordinate
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        pos = [fx, fy]
        
        # Debug: Print coordinates
        print(f"Face detected at X:{fx}, Y:{fy}")
        
        # Convert coordinate to servo degree
        # With horizontal flip, use normal mapping (not inverted)
        servoX = np.interp(fx, [0, ws], [0, 180])  # Normal mapping for flipped image
        
        # Y mapping - toggle with 'f' key if direction is wrong
        if flip_y_mapping:
            servoY = np.interp(fy, [0, hs], [180, 0])  # Inverted Y mapping
        else:
            servoY = np.interp(fy, [0, hs], [0, 180])  # Normal Y mapping
        
        # Debug: Print servo values before constraining
        print(f"Calculated Servo X:{servoX:.1f}, Y:{servoY:.1f}")
        
        # Constrain servo values
        servoX = max(0, min(180, servoX))
        servoY = max(0, min(180, servoY))
        
        # Debug: Print final servo values
        print(f"Final Servo X:{servoX:.1f}, Y:{servoY:.1f}")

        servoPos[0] = servoX
        servoPos[1] = servoY

        # Visual feedback
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    else:
        # No face detected - activate search mode
        led_pin.write(0)
        
        if not searching:
            searching = True
            search_cycles = 0  # Reset search cycles when starting search
            # Reset servo Y to 100 degrees when entering search mode
            servoPos[1] = 100
            servo_pinY.write(servoPos[1])
            print("Starting search mode... Servo Y reset to 100 degrees")
        
        # Improved search pattern: sweep X axis multiple times before changing Y
        servoPos[0] += search_direction_x * search_step
        
        # Check X boundaries and reverse direction
        if servoPos[0] >= 180:
            servoPos[0] = 180
            search_direction_x = -1
            search_cycles += 1
            
            # Only move Y after completing several X sweeps
            if search_cycles >= max_search_cycles:
                servoPos[1] += search_direction_y * (search_step * 2)
                search_cycles = 0  # Reset cycle counter
                
        elif servoPos[0] <= 0:
            servoPos[0] = 0
            search_direction_x = 1
            search_cycles += 1
            
            # Only move Y after completing several X sweeps
            if search_cycles >= max_search_cycles:
                servoPos[1] += search_direction_y * (search_step * 2)
                search_cycles = 0  # Reset cycle counter
        
        # Check Y boundaries and reverse direction
        if servoPos[1] >= 180:
            servoPos[1] = 180
            search_direction_y = -1
        elif servoPos[1] <= 0:
            servoPos[1] = 0
            search_direction_y = 1
        
        # Visual feedback for search mode
        cv2.putText(img, "SEARCHING...", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 255), 3)
        cv2.circle(img, (ws//2, hs//2), 80, (0, 255, 255), 2)
        cv2.circle(img, (ws//2, hs//2), 15, (0, 255, 255), cv2.FILLED)
        cv2.line(img, (0, hs//2), (ws, hs//2), (0, 0, 0), 2)  # x line
        cv2.line(img, (ws//2, hs), (ws//2, 0), (0, 0, 0), 2)  # y line
        
        # Add search delay
        time.sleep(search_delay)

    # Display servo positions and coordinates for debugging
    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    
    # Debug info - show face coordinates when detected
    if bboxs:
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        cv2.putText(img, f'Face X: {fx} Y: {fy}', (50, 200), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    
    # Display search status and cycle info
    if searching:
        cv2.putText(img, 'SEARCH MODE: ON', (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)
        cv2.putText(img, f'Search Cycles: {search_cycles}', (50, 350), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1)
    else:
        cv2.putText(img, 'TRACKING MODE: ON', (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
    
    # Display Y mapping status
    y_mode = "INVERTED" if flip_y_mapping else "NORMAL"
    cv2.putText(img, f'Y-Axis: {y_mode}', (50, 250), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
    cv2.putText(img, 'Press F to flip Y-axis', (50, 300), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)

    # Write to servos
    servo_pinX.write(servoPos[0])
    servo_pinY.write(servoPos[1])

    cv2.imshow("Image", img)
    
    # Keyboard controls
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('f'):
        flip_y_mapping = not flip_y_mapping
        print(f"Y-axis mapping: {'INVERTED' if flip_y_mapping else 'NORMAL'}")
    elif key == ord('r'):  # Reset servo to center position
        servoPos = [90, 90]
        servo_pinX.write(servoPos[0])
        servo_pinY.write(servoPos[1])
        print("Servo reset to center position")

# Cleanup
cap.release()
cv2.destroyAllWindows()
board.exit()
