import cv2
import numpy as np

COLORS = {
    "red": np.array([1, 230, 162]), # 0   
    "yellow": np.array([26, 201, 225]), # 2
    "blue": np.array([106, 149, 125]), # 4 
    "orange": np.array([13, 228, 180]), # 1
    "green": np.array([33, 181, 158]), # 3
    "black": np.array([25, 29, 52]), # 6     
    "purple": np.array([130, 116, 79]) # 5
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

def ensure_color_area_in_image(image_path, target_color, area_threshold=500):
    assert target_color in COLORS, f"{target_color} not found in COLORS dictionary."
    
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Image at path {image_path} could not be loaded.")

    # Convert image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)

    # Target color in HSV
    target_hsv = COLORS[target_color].astype(np.float32)

    # Compute the distance of each pixel to the target color in HSV space
    diff = hsv_image - target_hsv
    diff[:, :, 0] = np.minimum(np.abs(diff[:, :, 0]), 180 - np.abs(diff[:, :, 0]))  # Handle circular Hue
    distance = np.sqrt(np.sum(diff ** 2, axis=2))

    # Find the closest pixels
    closest_pixels = np.argsort(distance.ravel())[:area_threshold]
    indices = np.unravel_index(closest_pixels, distance.shape)

    # Compute the average color of the closest area
    closest_hsv_values = hsv_image[indices]
    average_closest_hsv = np.mean(closest_hsv_values, axis=0)

    # Compute global adjustment to bring the average to the target color
    adjustment = target_hsv - average_closest_hsv

    # Apply the adjustment globally
    adjusted_hsv_image = hsv_image + adjustment
    adjusted_hsv_image = np.clip(adjusted_hsv_image, [0, 0, 0], [180, 255, 255])

    # Convert back to BGR
    adjusted_image = cv2.cvtColor(adjusted_hsv_image.astype(np.uint8), cv2.COLOR_HSV2BGR)

    return adjusted_image, diff

def detect_color_positions(target_color, image_path, output_path, tolerance=(1, 10, 10)):
    assert target_color in COLORS

    image, _ = ensure_color_area_in_image(image_path, target_color) #cv2.imread(image_path)
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
print('detect which colors? [submit a non-color to be done]')
detect_colors = []
c = input()
while c in COLORS:
    detect_colors.append(c)
    c = input()

image_path = f'pictures/logitech_examples/example{example}.jpg'
for color in detect_colors:
    output_path = f'pictures/logitech_examples_output/example{example}_output_{color}.jpg'
    color_positions[color] = detect_color_positions(color, image_path, output_path)

    adjusted, diff = ensure_color_area_in_image(image_path, color)
    cv2.imwrite(f'pictures/logitech_examples_output/masks/adjusted_{color}.jpg', adjusted)
    cv2.imwrite(f'pictures/logitech_examples_output/masks/diff_{color}.jpg', diff)

colors_detected = [color for color in COLORS if color_positions[color] != []]

# print(color_positions)

initial_color_order = sorted(colors_detected, key=lambda c: color_positions[c][0][1])

print(initial_color_order)