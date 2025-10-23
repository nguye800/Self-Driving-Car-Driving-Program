from ultralytics import YOLO
import cv2

# Load YOLO model (you can use yolo11n.pt or yolov8m.pt)
model = YOLO("yolo11n.pt")

# Open webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break

    # Run inference
    results = model(frame)
    
    # Get the first result (frame prediction)
    boxes = results[0].boxes
    detections = []

    # Process each detected object
    for box in boxes:
        # Bounding box coordinates (x1, y1, x2, y2)
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # Object type
        class_id = int(box.cls[0])
        object_type = results[0].names[class_id]

        # Store result
        detections.append({
            "object_type": object_type,
            "bounding_box": (int(x1), int(y1), int(x2), int(y2)),
        })

        # Draw detections
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, object_type, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # Display annotated frame
    cv2.imshow("YOLO Live Detection", frame)

    # Print detections in this frame to console
    print(detections)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()