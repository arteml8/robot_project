import cv2

# GStreamer pipeline for CSI camera (IMX219/IMX477, adjust resolution/fps)
gst_pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, format=(string)BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=(string)BGR ! appsink"
)

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR!! Failed to open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("WARNING! Empty frame")
        break
    cv2.imshow("CSI Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()