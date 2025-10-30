from ultralytics import YOLO
import cv2
# from picamera2 import Picamera2
from depth_estimation import rpi_camera, compute_dispmap_sgbm, initialize_cam

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

        # find avg disparity of the box
        total = 0
        for x in range(int(x1), int(x2)):
            for y in range(int(y1), int(y2)):
                total += dispmap[y][x] # image arrays indexed in reverse
        box_disp = int(total // ((x2-x1)*(y2-y1)))

        # Store result
        # detections.append({
        #     "object_type": object_type,
        #     "bounding_box": (int(x1), int(y1), int(x2), int(y2)),
        #     "avg_disparity": box_disp,
        # })

        # Draw detections
        color = (255 - box_disp, 0, box_disp) # close red, far blue
        cv2.rectangle(frameL, (int(x1), int(y1)), (int(x2), int(y2)), color, -1)
        cv2.putText(frameL, object_type, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

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