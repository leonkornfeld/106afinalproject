import cv2
import numpy as np
import time

MAX_TIME = 3
# click_count = 0
# detect_colors, COLORS, hsv, image = None, None, None, None

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
    "green" : 1, 
    "blue" : 2,
    "purple" : 3,
    "black" : 6,
}

def capture_image(output_path, camera_index=0):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Error: Could not access the webcam.")
        return

    ret, frame = cap.read()

    if ret:
        cv2.imwrite(output_path, frame)
        print(f"Image saved at {output_path}")
    else:
        print("Error: Could not capture image.")

    cap.release()

def detect_color_positions(target_color, image_path, output_path, tolerance=np.array([10, 25, 25])):
    assert target_color in COLORS

    image = cv2.imread(image_path)

    height, width = image.shape[:2]
    mid_range_start = int(width * 0.25)
    mid_range_end = int(width * 0.75)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    base_hsv = COLORS[target_color].astype(np.uint32)
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
    cv2.imwrite(f'pictures/logitech_output/mask_{target_color}.jpg', mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    positions_with_areas = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        cx, cy = x + w // 2, y + h // 2
        if (area > 15): #and (mid_range_start <= cx <= mid_range_end):
            positions_with_areas.append(((cx, cy), area))

    positions_with_areas.sort(key=lambda item: item[1], reverse=True)
    positions = [pos for pos, _ in positions_with_areas]

    for i, pos in enumerate(positions):
        cx, cy = pos
        bgr_color = cv2.cvtColor(np.uint8([[base_hsv]]), cv2.COLOR_HSV2BGR)[0][0]
        bgr_color = tuple(int(c) for c in bgr_color)

        cv2.circle(image, (cx, cy), 10, bgr_color, -1)  
        cv2.putText(image, f"{i}. ({cx}, {cy})", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)

    cv2.imwrite(output_path, image)
    return positions

def get_color_order(image_path):

    color_positions = {color : [] for color in COLORS}

    def get_hsv_value(event, x, y, flags, param):
        nonlocal click_count

        if event == cv2.EVENT_LBUTTONDOWN and click_count < len(detect_colors):
            hsv_value = hsv[y, x]
            color_name = detect_colors[click_count]
            COLORS[color_name] = hsv_value.astype(np.uint32)
            print(f"Updated '{color_name}' to HSV value from ({x}, {y}): {hsv_value}")
            click_count += 1

    # print("Detect which colors? [submit a non-color to be done]")
    detect_colors = ['red', 'green', 'blue', 'purple'] # TODO: FIX THIS CODE
    # c = input()
    # while c in COLORS:
    #     detect_colors.append(c)
    #     c = input()

    color_counts = {}
    for color in detect_colors:
        color_counts[color] = color_counts.get(color, 0) + 1

    # calibrate_colors = input("Calibrate colors? [y/n]: ")
    if True: # calibrate_colors == "y": # TODO: FIX THIS
        click_count = 0
        image = cv2.imread(image_path)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("Image", 150, 350)
        cv2.setMouseCallback("Image", get_hsv_value)
        while click_count < len(detect_colors):
            cv2.resizeWindow("Image", 10, 10)
            cv2.imshow("Image", image)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                print("Exiting...")
                break

        cv2.destroyAllWindows()

    start_time = time.time()

    for color in set(detect_colors): # run the detection process on all types of colors listed in detect_colors
        output_path = f'pictures/logitech_output/output_{color}.jpg'
        
        tolerance = np.array([10, 25, 25])
        positions = detect_color_positions(color, image_path, output_path)

        while len(positions) != color_counts[color]:

            if time.time() - start_time > MAX_TIME and len(positions) > color_counts[color]:
                print(f"Exceeded maximum time allowance of {MAX_TIME} seconds. Giving up and returning best guesses for {color}")
                break

            if len(positions) < color_counts[color]: # increase tolerance
                tolerance += np.array([1, 0, 0])
                positions = detect_color_positions(color, image_path, output_path, tolerance=tolerance)   
                if len(positions) < color_counts[color]:
                    tolerance -= np.array([1, 0, 0])
                    tolerance += np.array([0, 1, 0])
                    positions = detect_color_positions(color, image_path, output_path, tolerance=tolerance)   
                if len(positions) < color_counts[color]:
                    tolerance -= np.array([0, 1, 0])
                    tolerance += np.array([0, 0, 1])
                    positions = detect_color_positions(color, image_path, output_path, tolerance=tolerance)   
                if len(positions) < color_counts[color]:
                    tolerance += 1
            elif len(positions) > color_counts[color]: # decrease tolerances
                tolerance -= np.array([1, 0, 0])
                positions = detect_color_positions(color, image_path, output_path, tolerance=tolerance)   
                if len(positions) < color_counts[color]:
                    tolerance += np.array([1, 0, 0])
                    tolerance -= np.array([0, 1, 0])
                    positions = detect_color_positions(color, image_path, output_path, tolerance=tolerance)   
                if len(positions) < color_counts[color]:
                    tolerance += np.array([0, 1, 0])
                    tolerance -= np.array([0, 0, 1])
                    positions = detect_color_positions(color, image_path, output_path, tolerance=tolerance)   
                if len(positions) < color_counts[color]:
                    tolerance -= 1

        color_positions[color] = positions

    colors_detected = [color for color in detect_colors if color_positions[color] != []]

    initial_color_order = sorted(colors_detected, key=lambda c: color_positions[c][0][1])

    return [COLOR_2_IDX[color] for color in initial_color_order]