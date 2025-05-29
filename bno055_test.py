import time
import board
import busio
import adafruit_bno055

# สร้างการเชื่อมต่อ I2C กับ BNO055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# ตั้ง sensor ให้อยู่ในโหมด Fusion เต็ม (Gyro + Accel + Mag)
sensor.mode = adafruit_bno055.NDOF_MODE

# แปลงเลขสถานะเป็นคำอธิบายง่ายๆ
def explain_status(name, level):
    symbols = ["✖", "⚠", "⚠", "✔"]
    descriptions = [
        f"{name}: {symbols[level]} Not calibrated",
        f"{name}: {symbols[level]} Low",
        f"{name}: {symbols[level]} Medium",
        f"{name}: {symbols[level]} ✅ Calibrated"
    ]
    return descriptions[level]

# วนลูปแสดงผล
while True:
    # อ่านค่าการ calibration
    sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status

    # แสดงสถานะแต่ละเซ็นเซอร์
    print("-" * 40)
    print("BNO055 Calibration Status")
    print(explain_status("System", sys_cal))
    print(explain_status("Gyroscope", gyro_cal))
    print(explain_status("Accelerometer", accel_cal))
    print(explain_status("Magnetometer", mag_cal))

    # แสดงค่า Yaw
    yaw = sensor.euler[0]
    if yaw is None:
        print("Yaw (°): ... waiting for sensor ...")
    else:
        print(f"Yaw (°): {yaw:.2f}")

    time.sleep(0.5)
