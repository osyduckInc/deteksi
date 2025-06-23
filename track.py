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
servoPos = [90.0, 90.0] # initial servo position (gunakan float untuk presisi)

# SIMPLE SMOOTHING VARIABLES - Lebih sederhana dan stabil
smoothing_factor = 0.15  # Nilai kecil untuk gerakan halus (0.1-0.3)
min_movement = 2.0       # Minimum pergerakan dalam derajat untuk mengurangi jitter

# Variables for searching mode
searching = False
search_direction_x = 1
search_direction_y = 1
search_step = 3
search_delay = 0.08

# Variables to track search pattern
search_cycles = 0
max_search_cycles = 3

# Variable to flip Y mapping
flip_y_mapping = False

# Initialize servo to center position
servo_pinX.write(int(servoPos[0]))
servo_pinY.write(int(servoPos[1]))
time.sleep(1)

def smooth_servo_movement(current_pos, target_pos, smoothing_factor, min_movement):
    """
    Smooth movement dengan validasi yang lebih ketat
    """
    # Hitung perbedaan
    diff_x = target_pos[0] - current_pos[0]
    diff_y = target_pos[1] - current_pos[1]
    
    # Terapkan minimum movement threshold
    if abs(diff_x) < min_movement:
        diff_x = 0
    if abs(diff_y) < min_movement:
        diff_y = 0
    
    # Smooth interpolation
    new_x = current_pos[0] + (diff_x * smoothing_factor)
    new_y = current_pos[1] + (diff_y * smoothing_factor)
    
    # Pastikan dalam range yang valid
    new_x = max(0.0, min(180.0, new_x))
    new_y = max(0.0, min(180.0, new_y))
    
    return [new_x, new_y]

print("Face tracking started...")

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Face detected
        searching = False
        search_cycles = 0
        led_pin.write(1)
        
        # Get face center
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        pos = [fx, fy]
        
        print(f"Face detected at X:{fx}, Y:{fy}")
        
        # Convert coordinate to servo degree
        target_servoX = np.interp(fx, [0, ws], [0, 180])
        
        if flip_y_mapping:
            target_servoY = np.interp(fy, [0, hs], [180, 0])
        else:
            target_servoY = np.interp(fy, [0, hs], [0, 180])
        
        # Constrain values
        target_servoX = max(0, min(180, target_servoX))
        target_servoY = max(0, min(180, target_servoY))
        
        target_pos = [float(target_servoX), float(target_servoY)]
        
        print(f"Target Servo X:{target_pos[0]:.1f}, Y:{target_pos[1]:.1f}")
        print(f"Current Servo X:{servoPos[0]:.1f}, Y:{servoPos[1]:.1f}")
        
        # Apply smooth movement - HANYA saat face tracking
        servoPos = smooth_servo_movement(servoPos, target_pos, smoothing_factor, min_movement)
        
        print(f"New Servo X:{servoPos[0]:.1f}, Y:{servoPos[1]:.1f}")
        print("---")
        
        # Visual feedback
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    else:
        # No face detected - activate search mode
        led_pin.write(0)
        
        if not searching:
            searching = True
            search_cycles = 0
            servoPos[1] = 100.0  # Reset Y langsung tanpa smoothing
            print("Starting search mode...")
        
        # Search movement - LANGSUNG tanpa smoothing untuk pencarian yang efektif
        servoPos[0] += search_direction_x * search_step
        
        # Check X boundaries
        if servoPos[0] >= 180:
            servoPos[0] = 180.0
            search_direction_x = -1
            search_cycles += 1
            
            if search_cycles >= max_search_cycles:
                servoPos[1] += search_direction_y * (search_step * 2)
                search_cycles = 0
                
        elif servoPos[0] <= 0:
            servoPos[0] = 0.0
            search_direction_x = 1
            search_cycles += 1
            
            if search_cycles >= max_search_cycles:
                servoPos[1] += search_direction_y * (search_step * 2)
                search_cycles = 0
        
        # Check Y boundaries
        if servoPos[1] >= 180:
            servoPos[1] = 180.0
            search_direction_y = -1
        elif servoPos[1] <= 0:
            servoPos[1] = 0.0
            search_direction_y = 1
        
        # Visual feedback
        cv2.putText(img, "SEARCHING...", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 255), 3)
        cv2.circle(img, (ws//2, hs//2), 80, (0, 255, 255), 2)
        cv2.circle(img, (ws//2, hs//2), 15, (0, 255, 255), cv2.FILLED)
        cv2.line(img, (0, hs//2), (ws, hs//2), (0, 0, 0), 2)
        cv2.line(img, (ws//2, hs), (ws//2, 0), (0, 0, 0), 2)
        
        time.sleep(search_delay)

    # Display information
    cv2.putText(img, f'Servo X: {servoPos[0]:.1f} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {servoPos[1]:.1f} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    
    if bboxs:
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        cv2.putText(img, f'Face X: {fx} Y: {fy}', (50, 200), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.putText(img, f'Target X: {target_pos[0]:.1f} Y: {target_pos[1]:.1f}', (50, 350), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1)
    
    if searching:
        cv2.putText(img, 'SEARCH MODE: ON', (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)
    else:
        cv2.putText(img, 'TRACKING MODE: ON', (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
    
    y_mode = "INVERTED" if flip_y_mapping else "NORMAL"
    cv2.putText(img, f'Y-Axis: {y_mode}', (50, 250), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
    cv2.putText(img, 'Press F to flip Y-axis', (50, 300), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
    cv2.putText(img, f'Smoothing: {smoothing_factor:.2f}', (50, 400), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
    cv2.putText(img, 'Press +/- to adjust smoothing', (50, 430), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)

    # Write to servos dengan pembulatan integer
    servo_pinX.write(int(round(servoPos[0])))
    servo_pinY.write(int(round(servoPos[1])))

    cv2.imshow("Image", img)
    
    # Keyboard controls
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('f'):
        flip_y_mapping = not flip_y_mapping
        print(f"Y-axis mapping: {'INVERTED' if flip_y_mapping else 'NORMAL'}")
    elif key == ord('r'):
        servoPos = [90.0, 90.0]
        servo_pinX.write(90)
        servo_pinY.write(90)
        print("Servo reset to center position")
    elif key == ord('=') or key == ord('+'):  # Increase smoothing
        smoothing_factor = min(1.0, smoothing_factor + 0.05)
        print(f"Smoothing factor: {smoothing_factor:.2f}")
    elif key == ord('-'):  # Decrease smoothing
        smoothing_factor = max(0.05, smoothing_factor - 0.05)
        print(f"Smoothing factor: {smoothing_factor:.2f}")
    elif key == ord('d'):  # Toggle debug mode
        print(f"Current servo position: X={servoPos[0]:.1f}, Y={servoPos[1]:.1f}")

# Cleanup
cap.release()
cv2.destroyAllWindows()
board.exit()
