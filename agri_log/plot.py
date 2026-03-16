import pandas as pd
import matplotlib.pyplot as plt

# 1. โหลดข้อมูลจากไฟล์ CSV
# ถ้าคุณบันทึกเป็นชื่ออื่น อย่าลืมแก้ชื่อไฟล์ตรงนี้นะครับ
df = pd.read_csv('/home/aorus-ubun/CMU/P.3/P.3-T.2/271392_Agriculture_Robot_Auto/agri_log/pid_log_20260316_104758.csv')

# 2. ตั้งค่าขนาดของกราฟ
plt.figure(figsize=(12, 6))

# 3. พล็อตเส้นกราฟ
# พล็อต target_right เป็นเส้นประสีน้ำเงิน
plt.plot(df['time'], df['target_right'], label='Target Right', linestyle='--', color='blue', linewidth=2)

# พล็อต curr_right เป็นเส้นทึบสีแดง
plt.plot(df['time'], df['curr_right'], label='Current Right', linestyle='-', color='red', linewidth=2)

# 4. ตกแต่งกราฟให้ดูง่ายขึ้น
plt.title('Motor Response: Target vs Current (Right)')
plt.xlabel('Time (seconds)')
plt.ylabel('Value')
plt.legend() # แสดงคำอธิบายเส้นกราฟ
plt.grid(True, linestyle=':', alpha=0.7) # เปิดเส้นตาราง

# 5. แสดงผลกราฟ
plt.show()