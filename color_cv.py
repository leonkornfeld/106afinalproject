import cv2
import numpy as np

def find_and_draw_colored_boxes(image_path, output_path):
    # Read the image
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Could not read the image.")
        return
    
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define color ranges for different colors in HSV
    color_ranges = {
        "red": [(np.array([0, 70, 50]), np.array([10, 255, 255])),
                (np.array([170, 50, 50]), np.array([180, 255, 255]))],
        "yellow": [(np.array([20, 100, 100]), np.array([30, 255, 255]))],
        "blue": [(np.array([100, 150, 50]), np.array([140, 255, 255]))],
        "orange": [(np.array([10, 100, 100]), np.array([20, 255, 255]))],
        "pink": [(np.array([140, 50, 50]), np.array([170, 255, 255]))],
        "green": [(np.array([40, 50, 50]), np.array([80, 255, 255]))]
    }

    # Dictionary to store colors for drawing bounding boxes
    box_colors = {
        "red": (0, 0, 255),
        "yellow": (0, 255, 255),
        "blue": (255, 0, 0),
        "orange": (0, 165, 255),
        "pink": (255, 105, 180),
        "green": (0, 255, 0)
    }

    # Loop through each color and detect objects
    for color_name, ranges in color_ranges.items():
        # Create a combined mask for the current color
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        
        # Perform morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours for the current color
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around the detected objects
        for contour in contours:
            # Get the bounding box for each contour
            x, y, w, h = cv2.boundingRect(contour)
            # Filter out small boxes
            if w > 20 and h > 20:  # Adjust size thresholds as needed
                cv2.rectangle(image, (x, y), (x + w, y + h), box_colors[color_name], 2)
                cv2.putText(image, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_colors[color_name], 2)

    # Save the result to the output path
    cv2.imwrite(output_path, image)
    print(f"Output saved to {output_path}")

# Example usage
image_path = 'test3.png'  # Replace with your input image path
output_path = 'output_image3.jpg'  # Replace with your desired output image path
find_and_draw_colored_boxes(image_path, output_path)