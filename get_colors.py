import cv2
import numpy as np

def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_value = hsv[y, x]
        print(f"HSV value at ({x}, {y}): {hsv_value}")

# Load the image
example = input('Use which example to test on? ')
image_path = f"pictures/logitech_examples/example{example}.jpg"  # Replace with your image path
image = cv2.imread(image_path)

if image is None:
    print("Error: Unable to load image.")
    exit()

# Convert the image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Display the image in a window
cv2.namedWindow("Image")
cv2.setMouseCallback("Image", get_hsv_value)

print("Click on a point in the image to get its HSV value. Press 'q' to quit.")

while True:
    cv2.imshow("Image", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

cv2.destroyAllWindows()