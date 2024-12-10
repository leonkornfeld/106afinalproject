import cv2
import numpy as np

COLORS = {
    "red": np.array([1, 230, 162]), 
    "orange": np.array([13, 228, 180]), 
    "yellow": np.array([26, 201, 225]),
    "green": np.array([33, 181, 158]),
    "blue": np.array([106, 149, 125]),
    "purple": np.array([130, 116, 79]),
    "black": np.array([25, 29, 52]),
}

COLOR_2_IDX = {
    "red" : 0,
    "orange" : 1,
    "yellow" : 2,
    "green" : 3, 
    "blue" : 4,
    "purple" : 5,
    "black" : 6,
}

def detect_color_positions(target_color, image_path, output_path, tolerance=(20, 100, 100)):
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

def get_hsv_value(event, x, y, flags, param):
    global click_count, detect_colors, COLORS, hsv, image

    if event == cv2.EVENT_LBUTTONDOWN and click_count < len(detect_colors):
        hsv_value = hsv[y, x]
        color_name = detect_colors[click_count]
        COLORS[color_name] = hsv_value
        print(f"Updated '{color_name}' to HSV value from ({x}, {y}): {hsv_value}")
        click_count += 1

print("Detect which colors? [submit a non-color to be done]")
detect_colors = []
c = input()
while c in COLORS:
    detect_colors.append(c)
    c = input()

calibrate_colors = input("Calibrate colors? [y/n]: ")
if calibrate_colors == "y":
    click_count = 0
    image = cv2.imread(image_path)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", get_hsv_value)
    while click_count < len(detect_colors):
        cv2.imshow("Image", image)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("Exiting...")
            break

    cv2.destroyAllWindows()
    print("Updated COLORS:", COLORS)

for color in detect_colors:
    output_path = f'pictures/logitech_examples_output/example{example}_output_{color}.jpg'
    color_positions[color] = detect_color_positions(color, image_path, output_path)

colors_detected = [color for color in detect_colors if color_positions[color] != []]

initial_color_order = sorted(colors_detected, key=lambda c: color_positions[c][0][1])

print(initial_color_order)