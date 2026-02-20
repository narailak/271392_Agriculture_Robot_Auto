import cv2
import os
import time

# ===================== CONFIG =====================
CAMERA_ID = 4
WIDTH = 1920
HEIGHT = 1080
FPS = 50
BASE_SAVE_DIR = "Record_data"
FOLDER_PREFIX = "captured"
# ==================================================

# 1. จัดการโฟลเดอร์ (หาเลขล่าสุด)
os.makedirs(BASE_SAVE_DIR, exist_ok=True)
existing_folders = [d for d in os.listdir(BASE_SAVE_DIR) if d.startswith(FOLDER_PREFIX)]
indices = [int(d.split('_')[-1]) for d in existing_folders if d.split('_')[-1].isdigit()]
folder_number = max(indices) + 1 if indices else 1

# สร้างโฟลเดอร์แบบ captured_001 เหมือนเดิมเพื่อให้เรียงลำดับในเครื่องง่าย
SAVE_DIR = os.path.join(BASE_SAVE_DIR, f"{FOLDER_PREFIX}_{folder_number:03d}")
os.makedirs(SAVE_DIR)

print(f"📁 Save to: {SAVE_DIR}")

cap = cv2.VideoCapture(CAMERA_ID)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

frame_count = 0
frame_interval = 1.0 / FPS
next_capture_time = time.time()

try:
    while True:
        ret, frame = cap.read()
        if not ret: break

        current_time = time.time()

        if current_time >= next_capture_time:
            # --- ปรับชื่อไฟล์ให้สั้น: {เลขโฟลเดอร์}_frame_{เลขเฟรม}.jpg ---
            # ตัวอย่าง: 1_frame_1.jpg, 1_frame_2.jpg ...
            file_id = f"{folder_number}_frame_{frame_count}.jpg"
            filename = os.path.join(SAVE_DIR, file_id)
            
            cv2.imwrite(filename, frame)
            
            next_capture_time += frame_interval
            frame_count += 1

        cv2.imshow("Recording (Press Q to stop)", cv2.resize(frame, (640, 360)))
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    cap.release()
    cv2.destroyAllWindows()
    print(f"✅ บันทึกสำเร็จ! ชื่อไฟล์ล่าสุดคือ: {folder_number}_frame_{frame_count-1}.jpg")