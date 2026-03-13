import cv2
from ultralytics import YOLO

# 1. โหลด Model YOLOv8n-seg (Instance Segmentation)
model = YOLO('/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/cabbage_detection/models/best (1).pt')

# 2. เปิดไฟล์วิดีโอ (เปลี่ยน 'path/to/your/video.mp4' เป็นที่อยู่ไฟล์ของคุณ)
# หากต้องการใช้กล้องเว็บแคม ให้เปลี่ยนเป็น 0
video_path = '/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/ros_ws/src/cabbage_detection/cabbage_detection/IMG_3512.mp4'
cap = cv2.VideoCapture(video_path)

# ตรวจสอบว่าเปิดไฟล์สำเร็จหรือไม่
if not cap.isOpened():
    print("Error: ไม่สามารถเปิดวิดีโอได้")
    exit()

while cap.isOpened():
    success, frame = cap.read()

    if success:
        # 3. รัน Inference บนเฟรมปัจจุบัน
        # persist=True ช่วยให้การ Track วัตถุในวิดีโอเสถียรขึ้น
        results = model.track(frame, persist=True)

        # 4. แสดงผลลัพธ์ (Plot Masks และ Boxes ลงบนเฟรม)
        annotated_frame = results[0].plot()

        # 5. แสดงหน้าต่างวิดีโอ
        cv2.imshow("YOLOv8 Segmentation", annotated_frame)

        # กด 'q' เพื่อออกจากลูป
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # สิ้นสุดวิดีโอ
        break

# คืนค่าทรัพยากร
cap.release()
cv2.destroyAllWindows()