from ultralytics import YOLO
import cv2
# from picamera2 import Picamera2
from depth_estimation import rpi_camera, compute_dispmap_sgbm, initialize_cam
import numpy as np

# Map disparity to color using expected depth range
MIN_DISP_THRESHOLD = 50   # Adjust based on your camera setup
MAX_DISP_THRESHOLD = 200

# Load YOLO model (you can use yolo11n.pt or yolov8m.pt)
model = YOLO("yolo11n.pt")

# Open webcam
capL = rpi_camera(1)
capR = rpi_camera(0)

# capL = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# capR = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# initialize_cam(capL)
# initialize_cam(capR)

while True:
    # get depth map
    frameL = capL.capture_array()
    frameR = capR.capture_array()
    # ret, frameL = capL.read()
    # ret, frameR = capL.read()
    grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

    dispmap = compute_dispmap_sgbm(grayL, grayR)

    # Run inference
    results = model(frameL)
    
    # Get the first result (frame prediction)
    boxes = results[0].boxes
    detections = []

    # Process each detected object
    for box in boxes:
        # Bounding box coordinates (x1, y1, x2, y2)
        # x1,y1 top left x2,y2 bottom right
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # Object type
        class_id = int(box.cls[0])
        object_type = results[0].names[class_id]

        # Find avg disparity of the box
        box_region = dispmap[int(y1):int(y2), int(x1):int(x2)]
        valid_disparities = box_region[box_region > 0]  # Filter out invalid pixels
        if valid_disparities.size == 0:
            continue  # No valid depth data in this box
        box_disp = int(valid_disparities.mean())

        # Clamp and normalize
        # Draw detections with outline only
        disp_normalized = np.clip((box_disp - MIN_DISP_THRESHOLD) / (MAX_DISP_THRESHOLD - MIN_DISP_THRESHOLD), 0, 1)
        color = (int((1-disp_normalized)*255), 0, int(disp_normalized*255))  # Red=close, Blue=far
        # color = (255 - box_disp, 0, box_disp)  # close=red, far=blue
        cv2.rectangle(frameL, (int(x1), int(y1)), (int(x2), int(y2)), color, 3)  # Outline only

        # Store result
        # detections.append({
        #     "object_type": object_type,
        #     "bounding_box": (int(x1), int(y1), int(x2), int(y2)),
        #     "avg_disparity": box_disp,
        # })
        
        # Add label with background for readability
        label = f"{object_type} d:{box_disp}"
        (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frameL, (int(x1), int(y1)-label_h-10), (int(x1)+label_w, int(y1)), color, -1)
        cv2.putText(frameL, label, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Display annotated frame
    cv2.imshow("YOLO Live Detection", frameL)

    # Print detections in this frame to console
    # print(detections)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capL.stop()
capR.stop()
cv2.destroyAllWindows()