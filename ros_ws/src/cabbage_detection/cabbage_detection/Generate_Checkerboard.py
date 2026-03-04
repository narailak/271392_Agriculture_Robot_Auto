import cv2
import numpy as np

# กำหนดจำนวนช่อง 10x7 (เพื่อให้ได้จุดตัดด้านใน 9x6 ตามที่โค้ดคาลิเบรตต้องการ)
squares_x = 10
squares_y = 7
square_size = 150  # ความละเอียด 150 พิกเซลต่อ 1 ช่อง (ภาพจะออกมาชัดเจนพอดีหน้าจอ)

# สร้างอาเรย์ตารางสีดำ
board = np.zeros((squares_y * square_size, squares_x * square_size), dtype=np.uint8)

# เติมสีขาวสลับช่อง
for i in range(squares_y):
    for j in range(squares_x):
        if (i + j) % 2 == 0:
            board[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size] = 255

# เพิ่มขอบสีขาวรอบๆ (สำคัญมาก: ช่วยให้ OpenCV แยกแยะขอบกระดานกับพื้นหลังได้ง่ายขึ้น)
border_size = 100
board_with_border = cv2.copyMakeBorder(
    board, border_size, border_size, border_size, border_size, 
    cv2.BORDER_CONSTANT, value=255
)

# บันทึกเป็นไฟล์ภาพ PNG
filename = 'checkerboard_ipad.png'
cv2.imwrite(filename, board_with_border)
print(f"สร้างไฟล์ {filename} เสร็จเรียบร้อย! ส่งเข้า iPad ได้เลยครับ")
