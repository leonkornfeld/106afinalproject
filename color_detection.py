import cv2
import numpy as np

COLORS = {
    "red": np.array([1, 230, 162]),       
    "yellow": np.array([26, 201, 225]),   
    "blue": np.array([106, 149, 125]),    
    "orange": np.array([13, 228, 180]),   
    "green": np.array([33, 181, 158]),    
    # "black": np.array([25, 29, 52]),         
    "purple": np.array([130, 116, 79])
}

def detect_color_positions(target_color, image_path, output_path, tolerance=(5, 25, 25)):
    assert target_color in COLORS

    image = cv2.imread(image_path)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    base_hsv = COLORS[target_color]
    lower_bound = np.array([
        max(0, base_hsv[0] - tolerance[0]),
        max(0, base_hsv[1] - tolerance[1]),
        max(0, base_hsv[2] - tolerance[2])
    ], dtype=np.uint8)
    upper_bound = np.array([
        min(180, base_hsv[0] + tolerance[0]),
        min(255, base_hsv[1] + tolerance[1]),
        min(255, base_hsv[2] + tolerance[2])
    ], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    cv2.imwrite(f'pictures/logitech_examples_output/masks/mask_{color}.jpg', mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    positions = []
    for contour in contours:
        if cv2.contourArea(contour) > 30:
            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w // 2, y + h // 2
            positions.append((cx, cy))

            bgr_color = cv2.cvtColor(np.uint8([[base_hsv]]), cv2.COLOR_HSV2BGR)[0][0]
            bgr_color = tuple(int(c) for c in bgr_color)

            cv2.circle(image, (cx, cy), 10, bgr_color, -1)  
            cv2.putText(image, f"({cx}, {cy})", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)


    cv2.imwrite(output_path, image)
    return positions

color_positions = {color : [] for color in COLORS}

example = input('Use which example to test on? ')

image_path = f'pictures/logitech_examples/example{example}.jpg'
for color in COLORS:
    output_path = f'pictures/logitech_examples_output/example{example}_output_{color}.jpg'
    color_positions[color] = detect_color_positions(color, image_path, output_path)

colors_detected = [color for color in COLORS if color_positions[color] != []]

# print(color_positions)

initial_color_order = sorted(colors_detected, key=lambda c: color_positions[c][0][1])

print(initial_color_order)