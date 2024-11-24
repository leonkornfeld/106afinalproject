import cv2
import numpy as np

COLOR_RANGES = {
    "red": [(np.array([0, 70, 50]), np.array([10, 255, 255])),
            (np.array([170, 50, 50]), np.array([180, 255, 255]))],
    # "yellow": [(np.array([20, 100, 100]), np.array([30, 255, 255]))],
    "blue": [(np.array([100, 150, 50]), np.array([140, 255, 255]))],
    "orange": [(np.array([10, 100, 100]), np.array([20, 255, 255]))],
    "pink": [(np.array([140, 50, 50]), np.array([170, 255, 255]))],
    "green": [(np.array([40, 50, 50]), np.array([80, 255, 255]))],
    "black": [(np.array([0, 0, 0]), np.array([180, 255, 30]))],  # Low V values for black
    "purple": [(np.array([125, 50, 50]), np.array([150, 255, 255]))]
}

BOX_COLORS = {
    "red": (0, 0, 255),
    # "yellow": (0, 255, 255),
    "blue": (255, 0, 0),
    "orange": (0, 165, 255),
    "pink": (255, 105, 180),
    "green": (0, 255, 0),
    "black": (0, 0, 0),
    "purple": (128, 0, 128)
}


def find_and_draw_colored_boxes(image, output_path):
        if image is None:
            print("Error: Could not read the image.")
            return
        
        image = cv2.imread(image)
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_order = []

        for color_name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 100 and h > 100: # and abs(h/w -1) <= .1:  


                    color_order.append((x, color_name))

                    cv2.rectangle(image, (x, y), (x + w, y + h), BOX_COLORS[color_name], 2)
                    cv2.putText(image, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOX_COLORS[color_name], 2)

        color_order = sorted(color_order)
        print(color_order)
        cv2.imwrite(output_path, image)

        return image, color_order



# Example usage
image_path = 'test_color_order.JPG'  # Replace with your input image path
output_path = 'output_image3.jpg'  # Replace with your desired output image path
find_and_draw_colored_boxes(image_path, output_path)