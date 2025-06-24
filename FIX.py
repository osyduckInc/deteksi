import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time

cap = cv2.VideoCapture(0)
# Reduce resolution for better performance on Raspberry Pi 3
ws, hs = 640, 480
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

port = "/dev/ttyACM1"
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
flip_y_mapping = False

# Initialize servo to center position
servo_pinX.write(servoPos[0])
servo_pinY.write(servoPos[1])
time.sleep(1)

print("Face tracking started... (Headless mode)")
print("Press Ctrl+C to stop")

try:
    while True:
        success, img = cap.read()
        
        if not success:
            print("Failed to read from camera")
            break
            
        img = cv2.flip(img, 1)
        
        img, bboxs = detector.findFaces(img, draw=False)

        if bboxs:
            # Face detected - turn off search mode and reset search variables
            searching = False
            search_cycles = 0  # Reset search cycle counter
            led_pin.write(1)
            
            # Get the coordinate
            fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
            
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

            servoPos[0] = servoX
            servoPos[1] = servoY
            
            # Print target position for debugging
            print(f"TARGET LOCKED - Position: ({fx}, {fy}) - Servo: ({servoX:.1f}, {servoY:.1f})")

        else:
            # No face detected - activate search mode
            led_pin.write(0)
            
            if not searching:
                searching = True
                search_cycles = 0  # Reset search cycles when starting search
                print("Starting search mode...")
            
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
            
            # Print search status for debugging
            print(f"SEARCHING... Servo: ({servoPos[0]:.1f}, {servoPos[1]:.1f})")
            
            # Add search delay
            time.sleep(search_delay)

        # Write to servos
        servo_pinX.write(servoPos[0])
        servo_pinY.write(servoPos[1])
        
        # Small delay to prevent overwhelming the system
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping face tracking...")

finally:
    # Cleanup
    cap.release()
    board.exit()
    print("Resources cleaned up. Goodbye!")
