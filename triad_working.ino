#include <Arduino_LSM9DS1.h>
#include "eigen.h"
#include <Eigen/Dense>

float accl_x, accl_y, accl_z;
float mag_x, mag_y, mag_z;
float gyro_x_off, gyro_y_off, gyro_z_off;
float accl_x_off, accl_y_off, accl_z_off;
float dt = 0.01;
using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;
using Eigen::AngleAxisf;
Matrix3f gyro_R;

void setup() {
  Serial.begin(115200);
  while (!Serial || !IMU.begin());
  Serial.println("Starting Calibration. Do not move IMU.");
  calibrate();
  // Serial.println("Calibration complete.");
  
  gyro_R.setIdentity();
}

void loop() {
  static uint last_time = micros();
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);
    Quaternionf gyro_quat = gyroIntegrate(x, y, z);

    IMU.readAcceleration(x, y, z);
    Vector3f accl((x - accl_x_off), (y - accl_y_off), z);
    accl.normalize();

    IMU.readMagneticField(x, y, z);
    Vector3f mag(x, y, z);

    // >>> UPDATED MAGNETO VALUES (ONLY CHANGE YOU REQUESTED) <<<
    Vector3f b(11.982825f, 28.755767f, -2.518753f);

    Matrix3f A;
    A << 1.246702f,  0.055814f, -0.009815f,
         0.055814f,  1.221789f, -0.062628f,
        -0.009815f, -0.062628f,  1.166521f;
    // <<< ---------------------------------------------------- >>>

    mag = A * (mag - b);

    mag[0] = -mag[0];   // you already had this
    // mag[2] = -mag[2];

    mag.normalize();


    float incl = 28.42743 * M_PI / 180.0f;
    float decl = -0.02141 * M_PI / 180.0f;
    Vector3f g_ref(0, 0, -1);
    // Vector3f mag_ref(cos(incl)*cos(decl), cos(incl)*sin(decl), sin(incl));
    // Serial.print("mag field: ");
    // Serial.print(mag_ref.x()); Serial.print(',');Serial.print(mag_ref.y());Serial.print(',');Serial.println(mag_ref.z());
    Vector3f mag_ref(0, 1, 0);
    Vector3f t1 = accl;
    Vector3f t2 = t1.cross(mag).normalized();
    Vector3f t3 = t1.cross(t2);
    Matrix3f M_body;
    M_body.col(0) = t1;
    M_body.col(1) = t2;
    M_body.col(2) = t3;

    Vector3f r1 = g_ref;
    Vector3f r2 = r1.cross(mag_ref).normalized();
    Vector3f r3 = r1.cross(r2);
    Matrix3f M_ref;
    M_ref << r1, r2, r3;
    Matrix3f triad_R = M_ref * M_body.transpose();
    Quaternionf triad_quat(triad_R);


    // Serial.print(gyro_quat.w()); Serial.print(',');
    // Serial.print(gyro_quat.x()); Serial.print(',');
    // Serial.print(gyro_quat.y()); Serial.print(',');
    // Serial.print(gyro_quat.z()); Serial.print(',');
    
    Serial.print(triad_quat.w()); Serial.print(',');
    Serial.print(triad_quat.x()); Serial.print(',');
    Serial.print(triad_quat.y()); Serial.print(',');
    Serial.println(triad_quat.z());
    
  }

  while (micros() - last_time < 10000);
  last_time += 10000;
}

void calibrate() {
  int c = 0;
  uint last_time = micros();
  while (c < 2000) {
    while (!IMU.gyroscopeAvailable() || !IMU.accelerationAvailable());
    float x, y, z;

    IMU.readGyroscope(x, y, z);
    gyro_x_off += x; gyro_y_off += y; gyro_z_off += z;

    IMU.readAcceleration(x, y, z);
    accl_x_off += x; accl_y_off += y; accl_z_off += z;

    c++;
    while (micros() - last_time < 100);
    last_time += 100;
  }

  gyro_x_off /= 2000; gyro_y_off /= 2000; gyro_z_off /= 2000;
  accl_x_off /= 2000; accl_y_off /= 2000; accl_z_off /= 2000;
}

Quaternionf gyroIntegrate(float x, float y ,float z) {
  Vector3f omega(x-gyro_x_off, -(y-gyro_y_off), z-gyro_z_off);
  omega *= M_PI / 180.0f;
  float theta = dt * omega.norm();
  Matrix3f K;
  K << 0, -omega[2], omega[1], omega[2], 0, -omega[0], -omega[1], omega[0], 0;
  K *= dt / theta; 
  Matrix3f exp_omega = Matrix3f::Identity() + K * sin(theta) + K*K * (1 - cos(theta));
  gyro_R *= exp_omega;

  Quaternionf quat(gyro_R);
  return quat;
}