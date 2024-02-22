from picamera import PiCamera
import cv2

# Inisiasi kamera
camera = PiCamera()

# Set resolusi kamera 
camera.resolution = (640, 480)

# Set framerate kamera
camera.framerate = 32

# Membuka preview kamera
camera.start_preview() 

# Inisiasi haar cascade untuk deteksi wajah
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Loop untuk membaca frame secara realtime
while True:

  # Ambil frame dari kamera
  camera.capture(rawCapture, format="bgr")
  image = rawCapture.array

  # Ubah ke grayscale
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  
  # Deteksi wajah
  faces = face_cascade.detectMultiScale(gray, 1.3, 5)

  # Gambar kotak pada wajah
  for (x,y,w,h) in faces:
    cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2) 

  # Tampilkan hasil    
  cv2.imshow('frame', image)

  # Jeda 10ms sebelum ambil frame berikutnya
  key = cv2.waitKey(10)
  if key == 27:
    break

# Membersihkan dan stop kamera
cv2.destroyAllWindows()
camera.stop_preview()
