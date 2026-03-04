import cv2
import numpy as np

# ==========================================
# ตั้งค่าตารางหมากรุก (Checkerboard)
# ==========================================
# จำนวน "จุดตัดด้านใน" ของกระดาน (แนวนอน x แนวตั้ง) 
# ถ้ากระดานมี 10x7 ช่อง จุดตัดด้านในคือ 9x6
CHECKERBOARD = (9, 6)

# เกณฑ์การหยุดหาจุด (Criteria) เพื่อความแม่นยำระดับ Sub-pixel
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# เตรียมพิกัด 3D ของจุดบนกระดาน (0,0,0), (1,0,0), (2,0,0) ...
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# อาเรย์สำหรับเก็บจุด 3D และ 2D จากรูปภาพทั้งหมด
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# ==========================================
# เปิดกล้อง
# ==========================================
# ลองเปลี่ยนเลข 0 เป็น 2 หรือ 4 ถ้าระบบหาแพ็กเกจกล้องไม่เจอ (ขึ้นอยู่กับพอร์ตบน Ubuntu)
cap = cv2.VideoCapture(4)

print("========================================")
print("กด 'c' เพื่อ Capture ถ่ายภาพตาราง (ต้องการประมาณ 15-20 ภาพ ในมุมต่างๆ)")
print("กด 'q' เพื่อ Quit และเริ่มคำนวณ Camera Matrix")
print("========================================")

captured_images = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("ไม่สามารถอ่านภาพจากกล้องได้")
        break
        
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # วาดหน้าต่างกล้อง
    display_frame = frame.copy()

    # ลองหาจุดตัดของตารางหมากรุกแบบ Real-time
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret_corners:
        # วาดเส้นทับลงไปเพื่อบอกว่าระบบมองเห็นตาราง
        cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_corners)
        cv2.putText(display_frame, "Checkerboard Detected! Press 'c' to capture", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(display_frame, "Searching for checkerboard...", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
    cv2.putText(display_frame, f"Captured: {captured_images}/20", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow('Camera Calibration', display_frame)
    
    key = cv2.waitKey(1) & 0xFF
    
    # ถ้ากด 'c' และมองเห็นตาราง ให้บันทึกจุดไว้
    if key == ord('c') and ret_corners:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        captured_images += 1
        print(f"บันทึกภาพที่ {captured_images} เรียบร้อย!")
        
    # ถ้ากด 'q' ให้ออกจากลูปเพื่อไปคำนวณ
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# ==========================================
# เริ่มการคำนวณ Camera Matrix
# ==========================================
if captured_images > 0:
    print("\nกำลังคำนวณ พารามิเตอร์ของกล้อง (กรุณารอสักครู่)...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\n================ ผลลัพธ์ ================")
    # ดึงค่าพารามิเตอร์ออกมา
    fx = mtx[0, 0]
    fy = mtx[1, 1]
    cx = mtx[0, 2]
    cy = mtx[1, 2]

    print(f"Camera Matrix (mtx): \n{mtx}")
    print(f"Distortion Coefficients (dist): \n{dist}")
    
    print("\n*** นำตัวเลขด้านล่างนี้ ไปใส่ในโค้ด ROS 2 ของคุณ ***")
    print(f"self.camera_params = [{fx:.2f}, {fy:.2f}, {cx:.2f}, {cy:.2f}]")
    print("=========================================")
else:
    print("คุณยังไม่ได้บันทึกภาพตารางหมากรุกเลย ยกเลิกการคำนวณ")