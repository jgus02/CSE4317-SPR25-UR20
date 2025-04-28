import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import RPi.GPIO as GPIO

# GPIO setup
GPIO_PIN = 17 
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT)

# Load TFLite model
interpreter = tflite.Interpreter(model_path="detect.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()


# Initialize OpenCV camera
cap = cv2.VideoCapture(0)  # Use default camera

UR20_DIST = 167.64 # cm (66 inches)
AVG_PERSON_HEIGHT = 170 #cm
PIXEL_HEIGHT = 150

focal_length = (PIXEL_HEIGHT * UR20_DIST) / AVG_PERSON_HEIGHT

def estimate_dist(pixel_h):
    return (AVG_PERSON_HEIGHT * focal_length ) / pixel_h
 
def detect_human(frame):
    # Resize frame to model input size (300x300)
    input_data = cv2.resize(frame, (300, 300))
    input_data = np.expand_dims(input_data, axis=0).astype(np.uint8)

    # Run inference
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    # Get detection results
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]  # Bounding boxes
    classes = interpreter.get_tensor(output_details[1]['index'])[0]  # Class IDs
    scores = interpreter.get_tensor(output_details[2]['index'])[0]  # Confidence scores
    
    count = 0
    for i in range(len(scores)):
        if scores[i] > 0.63 and int(classes[i]) == 0:  # Class 0 = person
            # Extract box coordinates
            ymin, xmin, ymax, xmax = boxes[i]
            xmin, xmax, ymin, ymax = int(xmin * frame.shape[1]), int(xmax * frame.shape[1]), int(ymin * frame.shape[0]), int(ymax * frame.shape[0])

            middle = int(frame.shape[1] * 0.3)
            right = int(frame.shape[1] * 0.6)
            box_center = (xmin + xmax) // 2 

            px_h = ymax - ymin
            
            if estimate_dist(px_h) <= UR20_DIST and middle <= box_center <= right:
                count += 1
                
            #draw the region in which we are detecting people in view
            cv2.line(frame, (middle, 0), (middle, frame.shape[0]), (255, 255, 0), 2)
            cv2.line(frame, (right, 0), (right, frame.shape[0]), (255,255, 0), 2)
 
            # Draw bounding box & label
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0) if count > 0 else (0, 0, 255), 2)
            label = f"Human: {int(scores[i] * 100) }%"
            cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame, count

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame, count = detect_human(frame)

        # Set GPIO HIGH or LOW

        GPIO.output(GPIO_PIN, GPIO.HIGH if count > 0 else GPIO.LOW)
        # Show the video feed with detections
        cv2.imshow("Human Detection", frame)
        print(count)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
