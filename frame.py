import cv2
import numpy as np

def extract_8mm_frames(image_path, output_prefix="frame"):
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 1. Threshold to highlight sprocket holes (adjust threshold value if needed)
    _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)

    # 2. Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sprockets = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = float(w) / h
        
        # Filter by dimensions matching the sprocket size in your resolution
        if 15 < w < 60 and 15 < h < 60 and 0.7 < aspect_ratio < 1.4:
            sprockets.append((x, y, w, h))

    # Sort top-to-bottom
    sprockets = sorted(sprockets, key=lambda s: s[1])

    # 3. Crop relative to sprockets
    for idx, (sx, sy, sw, sh) in enumerate(sprockets):
        # Calculate center of sprocket
        cx, cy = sx + sw // 2, sy + sh // 2

        # Define frame offsets relative to sprocket center (adjust for your scan)
        # Super 8: Frame is horizontally aligned with sprocket
        # Regular 8: Frame line sits between sprockets
        frame_w = 400   # frame width in pixels
        frame_h = 300   # frame height in pixels
        offset_x = 50   # distance from sprocket center to left edge of frame
        offset_y = -150 # vertical offset from sprocket center to top edge of frame

        x1 = cx + offset_x
        y1 = cy + offset_y
        x2 = x1 + frame_w
        y2 = y1 + frame_h

        # Bounds check and crop
        if y1 >= 0 and y2 <= img.shape[0] and x1 >= 0 and x2 <= img.shape[1]:
            crop = img[y1:y2, x1:x2]
            cv2.imwrite(f"{output_prefix}_{idx:04d}.png", crop)

# Usage
extract_8mm_frames("scan_strip_01.png")
