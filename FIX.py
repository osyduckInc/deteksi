import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time
import serial.tools.list_ports

def find_arduino_port():
    """Automatically find Arduino port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Check for common Arduino descriptions
        if any(keyword in port.description.upper() for keyword in ['ARDUINO', 'CH340', 'CP210', 'FTDI', 'USB SERIAL']):
            return port.device
    
    # If no Arduino found, show available ports
    available_ports = [port.device for port in ports]
    if available_ports:
        print(f"Available ports: {available_ports}")
        print("Please check which port your Arduino is connected to")
        return available_ports[0]  # Return first available port
    
    return None

def connect_arduino(max_retries=3):
    """Connect to Arduino with retry mechanism"""
    for attempt in range(max_retries):
        try:
            port = find_arduino_port()
            if not port:
                print(f"Attempt {attempt + 1}: No Arduino port found")
                time.sleep(2)
                continue
                
            print(f"Attempt {attempt + 1}: Connecting to {port}")
            board = pyfirmata.Arduino(port)
            time.sleep(3)  # Wait longer for Arduino to initialize
            
            # Test connection by blinking built-in LED
            try:
                board.digital[13].write(1)
                time.sleep(0.2)
                board.digital[13].write(0)
                print("Arduino connected successfully!")
                return board
            except:
                print("Failed to communicate with Arduino")
                continue
                
        except Exception as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            time.sleep(2)
    
    print("Failed to connect to Arduino after all attempts")
    return None

def safe_servo_write(servo_pin, angle, max_retries=3):
    """Safely write to servo with error handling"""
    for attempt in range(max_retries):
        try:
            servo_pin.write(angle)
            return True
        except Exception as e:
            print(f"Servo write failed (attempt {attempt + 1}): {e}")
            time.sleep(0.1)
    return False

def main():
    # Initialize camera
    cap = cv2.VideoCapture(0)
    ws, hs = 640, 480
    cap.set(3, ws)
    cap.set(4, hs)

    if not cap.isOpened():
        print("Camera couldn't access!!!")
        return

    # Connect to Arduino
    board = connect_arduino()
    if not board:
        print("Cannot connect to Arduino. Exiting...")
        cap.release()
        return

    try:
        # Initialize pins
        servo_pinX = board.get_pin('d:9:s')  # pin 9 Arduino
        servo_pinY = board.get_pin('d:10:s')  # pin 10 Arduino
        led_pin = board.get_pin('d:5:o')  # Pin 5 Arduino

        detector = FaceDetector()
        servoPos = [90, 90]  # initial servo position

        # Variables for searching mode
        searching = False
        search_direction_x = 1
        search_direction_y = 1
        search_step = 5
        search_delay = 0.1
        search_cycles = 0
        max_search_cycles = 3
        flip_y_mapping = False

        # Initialize servo to center position
        safe_servo_write(servo_pinX, servoPos[0])
        safe_servo_write(servo_pinY, servoPos[1])
        time.sleep(1)

        print("Face tracking started... (Headless mode)")
        print("Press Ctrl+C to stop")

        while True:
            success, img = cap.read()
            
            if not success:
                print("Failed to read from camera")
                break
                
            img = cv2.flip(img, 1)
            img, bboxs = detector.findFaces(img, draw=False)

            if bboxs:
                # Face detected
                searching = False
                search_cycles = 0
                
                try:
                    led_pin.write(1)
                except:
                    print("Failed to write to LED")
                
                # Get coordinates
                fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
                
                # Convert to servo angles
                servoX = np.interp(fx, [0, ws], [0, 180])
                
                if flip_y_mapping:
                    servoY = np.interp(fy, [0, hs], [180, 0])
                else:
                    servoY = np.interp(fy, [0, hs], [0, 180])
                
                # Constrain values
                servoX = max(0, min(180, servoX))
                servoY = max(0, min(180, servoY))

                servoPos[0] = servoX
                servoPos[1] = servoY
                
                print(f"TARGET LOCKED - Position: ({fx}, {fy}) - Servo: ({servoX:.1f}, {servoY:.1f})")

            else:
                # No face detected - search mode
                try:
                    led_pin.write(0)
                except:
                    pass
                
                if not searching:
                    searching = True
                    search_cycles = 0
                    print("Starting search mode...")
                
                # Search pattern
                servoPos[0] += search_direction_x * search_step
                
                if servoPos[0] >= 180:
                    servoPos[0] = 180
                    search_direction_x = -1
                    search_cycles += 1
                    
                    if search_cycles >= max_search_cycles:
                        servoPos[1] += search_direction_y * (search_step * 2)
                        search_cycles = 0
                        
                elif servoPos[0] <= 0:
                    servoPos[0] = 0
                    search_direction_x = 1
                    search_cycles += 1
                    
                    if search_cycles >= max_search_cycles:
                        servoPos[1] += search_direction_y * (search_step * 2)
                        search_cycles = 0
                
                # Y boundaries
                if servoPos[1] >= 180:
                    servoPos[1] = 180
                    search_direction_y = -1
                elif servoPos[1] <= 0:
                    servoPos[1] = 0
                    search_direction_y = 1
                
                print(f"SEARCHING... Servo: ({servoPos[0]:.1f}, {servoPos[1]:.1f})")
                time.sleep(search_delay)

            # Write to servos with error handling
            safe_servo_write(servo_pinX, servoPos[0])
            safe_servo_write(servo_pinY, servoPos[1])
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping face tracking...")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Cleanup
        cap.release()
        try:
            board.exit()
        except:
            pass
        print("Resources cleaned up. Goodbye!")

if __name__ == "__main__":
    main()
