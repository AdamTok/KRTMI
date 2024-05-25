from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
import cv2
# import serial

# Initialize YOLOv8 model
model = YOLO("F:\KRTMI11\yolo5-object-detection-and-centroid-finding-main\sampah1-obb.pt")
names = model.names

# Open video file
cap = cv2.VideoCapture(0)
assert cap.isOpened(), ""
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

while cap.isOpened():
    success, im0 = cap.read()

    if success:
        # Make predictions on each frame
        results = model.track(im0)
        pred_boxes = results[0].obb

        # Initialize Annotator for visualization
        annotator = Annotator(im0, line_width=2, example=names)

        # Iterate over predicted bounding boxes and draw on image
        for d in reversed(pred_boxes):
            class_label = names[int(d.cls)]
            box = d.xyxyxyxy.reshape(-1, 4, 2).squeeze()

            # Calculate centroid of bounding box
            center_x = int((box[0][0] + box[2][0]) / 2)
            center_y = int((box[0][1] + box[2][1]) / 2)
            centroid = (center_x, center_y)
            print("BBox Coordinates : ", class_label, box, "Center of the bbox:", centroid)

            # Get detected object label
            annotator.box_label(box, class_label, color=colors(int(d.cls), True), rotated=True)

            # Draw centroid on image
            cv2.circle(im0, centroid, 5, (255, 0, 0), -1)

            # Send centroid and class label via serial port
            # ser.write(f"{centroid[0]},{centroid[1]},{class_label}\n".encode())

        # Display annotated image
        cv2.imshow("ultralytics", im0)

        # Check for key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    break

# Release video capture and close windows
cap.release()
cv2.destroyAllWindows()
