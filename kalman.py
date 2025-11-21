import serial
import math
import time

class KalmanFilter:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0, 0], [0, 0]]

    def update(self, newAngle, newRate, dt):
        rate = newRate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[1][0] - self.P[0][1] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        y = newAngle - self.angle
        self.angle += K[0] * y
        self.bias  += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle


# ---- Main Script ----
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # adjust port
kf_pitch = KalmanFilter()
kf_roll  = KalmanFilter()
kf_yaw   = KalmanFilter()

prev_time = time.time()

while True:
    raw = ser.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if not line or "," not in line:
        continue

    try:
        ax, ay, az, gx, gy, gz, temp = map(float, line.split(","))
    except ValueError:
        continue

    # --- Accelerometer angles ---
    # Pitch (rotation around X-axis)
    acc_pitch = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
    # Roll (rotation around Y-axis)
    acc_roll  = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    # No reliable acc_yaw without magnetometer

    # --- Gyro rates (rad/s → deg/s) ---
    gyro_pitch_rate = math.degrees(gx)
    gyro_roll_rate  = math.degrees(gy)
    gyro_yaw_rate   = math.degrees(gz)

    # --- Time step ---
    now = time.time()
    dt = now - prev_time
    prev_time = now

    # --- Kalman Updates ---
    kalman_pitch = kf_pitch.update(acc_pitch, gyro_pitch_rate, dt)
    kalman_roll  = kf_roll.update(acc_roll, gyro_roll_rate, dt)
    # Yaw will drift (no accelerometer reference)
    kalman_yaw   = kf_yaw.update(0.0, gyro_yaw_rate, dt)

    print(f"Pitch: {kalman_pitch:.2f}°, Roll: {kalman_roll:.2f}°, Yaw: {kalman_yaw:.2f}°")git 