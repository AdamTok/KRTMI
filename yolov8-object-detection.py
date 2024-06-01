from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
import cv2
import RPi.GPIO as GPIO
import time

# Nonaktifkan peringatan GPIO
GPIO.setwarnings(False)

# Set the GPIO mode jika belum diatur
try:
    GPIO.setmode(GPIO.BCM)
except ValueError as e:
    print(f"Mode GPIO sudah diatur: {e}")

# Set up GPIO pin 17 as an output for buzzer
dat1 = 17
dat2 = 27
dat3 = 22
dat4 = 23
dat5 = 24

if GPIO.getmode() is None:
    GPIO.setmode(GPIO.BCM)
GPIO.setup(dat1, GPIO.OUT)
GPIO.setup(dat2, GPIO.OUT)
GPIO.setup(dat3, GPIO.OUT)
GPIO.setup(dat4, GPIO.OUT)
GPIO.setup(dat5, GPIO.OUT)

def send1(duration):
    """Fungsi untuk menyalakan buzzer selama `duration` detik"""
    GPIO.output(dat1, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(dat1, GPIO.LOW)
def send2(duration):
    """Fungsi untuk menyalakan buzzer selama `duration` detik"""
    GPIO.output(dat2, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(dat2, GPIO.LOW)
def send3(duration):
    """Fungsi untuk menyalakan buzzer selama `duration` detik"""
    GPIO.output(dat3, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(dat3, GPIO.LOW)
def send4(duration):
    """Fungsi untuk menyalakan buzzer selama `duration` detik"""
    GPIO.output(dat4, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(dat4, GPIO.LOW)
def send5(duration):
    """Fungsi untuk menyalakan buzzer selama `duration` detik"""
    GPIO.output(dat5, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(dat5, GPIO.LOW)

# Initialize YOLOv8 model
model = YOLO("sampah1n-obb.pt")
names = model.names

# Open video file
cap = cv2.VideoCapture(0)
assert cap.isOpened(), "Failed to open camera"

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

            # Check if the detected object is "koran" and send '2' to Arduino
            if class_label == "Daunpisangkering":
                send1(7)

            elif class_label == "Daungedang":
                send1(7)

            elif class_label == "Kertasminyak":
                send2(7)
                
            elif class_label == "Koran":
                send2(7)

            elif class_label == "Botolplastik":
                send3(7)

            elif class_label == "Mikahijau":
                send3(7)

            elif class_label == "Mikabening":
                send3(7)

            elif class_label == "Seng":
                send4(7)

            elif class_label == "tembaga":
                send5(7)

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
GPIO.cleanup()

# Close serial connection

# const int pin53 = 53;  
# const int pin51 = 51;  
# const int pin49 = 49;  
# const int pin47 = 47;  
# const int pin45 = 45;  

# void setup() {
#     Serial.begin(115200);  
#     pinMode(pin53, INPUT);   
#     pinMode(pin51, INPUT);   
#     pinMode(pin49, INPUT);   
#     pinMode(pin47, INPUT);   
#     pinMode(pin45, INPUT);   
# }

# void loop() {
#     int signal53 = digitalRead(pin53);  
#     int signal51 = digitalRead(pin51);  
#     int signal49 = digitalRead(pin49);  
#     int signal47 = digitalRead(pin47);  
#     int signal45 = digitalRead(pin45);  

#     if (signal53 == HIGH) {
#         Serial.println("D53 Received HIGH signal");
#         delay(1000);
#     } else if (signal53 == LOW) {
#         Serial.println("D53 Received LOW signal");
#         delay(1000);
#     }

#     if (signal51 == HIGH) {
#         Serial.println("D51 Received HIGH signal");
#         delay(1000);
#     } else if (signal51 == LOW) {
#         Serial.println("D51 Received LOW signal");
#         delay(1000);
#     }

#     if (signal49 == HIGH) {
#         Serial.println("D49 Received HIGH signal");
#         delay(1000);
#     } else if (signal49 == LOW) {
#         Serial.println("D49 Received LOW signal");
#         delay(1000);
#     }

#     if (signal47 == HIGH) {
#         Serial.println("D47 Received HIGH signal");
#         delay(1000);
#     } else if (signal47 == LOW) {
#         Serial.println("D47 Received LOW signal");
#         delay(1000);
#     }

#     if (signal45 == HIGH) {
#         Serial.println("D45 Received HIGH signal");
#         delay(1000);
#     } else if (signal == LOW) {
#         Serial.println("D45 eceived LOW signal");
#         delay(1000);
#     }

#     delay(50);  // Tambahkan delay untuk menghindari flooding serial monitor
# }
