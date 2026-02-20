import cv2
from ultralytics import YOLO
import os

# ===== โหลดโมเดล =====
model = YOLO("/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/cabbage_detection/models/Garden.pt")

# ===== เปิดกล้อง (0 = กล้องหลัก) =====
cap = cv2.VideoCapture(4)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

print("✅ Garden YOLO Started (Press ESC to exit)")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Cannot read frame")
        break

    # ===== Run Detection =====
    results = model(frame, conf=0.5)

    # วาด bounding box ทีละ object
    for r in results:
        boxes = r.boxes

        if boxes is None:
            continue

        for box in boxes:
            # พิกัดกรอบ
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # confidence
            conf = float(box.conf[0])

            # class id
            cls_id = int(box.cls[0])
            label = model.names[cls_id]

            # ===== วาดกรอบ =====
            cv2.rectangle(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2
            )

            # ===== ใส่ข้อความ =====
            text = f"{label} {conf:.2f}"
            cv2.putText(
                frame,
                text,
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

    # ===== แสดงผล =====
    cv2.imshow("Garden YOLO Detection", frame)

    # กด ESC เพื่อออก
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()