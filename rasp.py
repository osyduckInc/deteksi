from picamera import PiCamera
import time
import cv2

# Inisiasi kamera
camera = PiCamera() 
# Set resolusi kamera 
camera.resolution = (640, 480)
# Set framerate kamera
camera.framerate = 32
# Membuka preview kamera 
camera.start_preview()
time.sleep(2)
# Ambil foto dan simpan
camera.capture('/home/pi/image.jpg') 

# Loop untuk stream video
while True:

  # Ambil frame gambar dari kamera
  camera.capture('/home/pi/frame.jpg')  

  # Baca frame gambar
  image = cv2.imread('/home/pi/frame.jpg') 

  # Lakukan proses OpenCV pada frame di sini

  # Tampilkan gambar
  cv2.imshow('frame', image)

  # Deteksi tombol ESC untuk berhenti
  if cv2.waitKey(1) == 27:
    break

# Hentikan preview kamera  
camera.stop_preview()
