#include <Arduino_LSM9DS1.h>
#include "eigen.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

// =======================================================
// OUTPUT CONFIG
// Set TRUE -> print TRIAD-only quaternion (skip filter)
// Set FALSE -> run full Direct Complementary Filter and print fused quaternion
// =======================================================
const bool OUTPUT_TRIAD_ONLY = true; // <<---- change this flag

// --- Tuning Parameters ---
const float kP = 0.05f;   // Proportional gain (trusts TRIAD/AM correction)
const float kI = 0.1f;   // Integral gain (learns gyro bias)
const float DEG_TO_RADIAN = M_PI / 180.0f;

// --- LOCATION CONFIGURATION ---
// (Not used now for mag inertial vector, kept for other uses)
const float MAGNETIC_DIP_DEG = 18.0f; 

// --- Calibration Globals ---
Vector3f accel_bias = Vector3f::Zero();
Vector3f gyro_bias  = Vector3f::Zero();

// --- Magnetometer Hard/Soft Iron Calibration ---
// Use the b and A_inv values you provided earlier
Vector3f mag_b(27.250990f, 41.956763f, -17.595884f);

// Direct inverse matrix A_inv (A^-1)
Matrix3f mag_A_inv;

// --- Filter State ---
Matrix3f R_est = Matrix3f::Identity(); // Estimated Rotation Matrix (Body -> Inertial)
Vector3f b_est = Vector3f::Zero();     // Dynamic Gyro Bias Estimate
unsigned long last_time_us = 0;

// --- Function Declarations ---
void printQuaternionFlat(const Quaternionf& q);
Matrix3f getTriadRotation(Vector3f accel_meas, Vector3f mag_meas);
void calibrateSensors();
Matrix3f rodrigues_exp(Vector3f w, float dt);
Vector3f vex(Matrix3f M);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // --- Directly assign A_inv (no inverse() call) ---
  // Values taken from your earlier message / screenshot:
  mag_A_inv.setZero();
  mag_A_inv(0,0) = 1.888865f;  mag_A_inv(0,1) = 0.049203f;  mag_A_inv(0,2) = 0.005467f;
  mag_A_inv(1,0) = 0.049203f;  mag_A_inv(1,1) = 1.830249f;  mag_A_inv(1,2) = -0.109767f;
  mag_A_inv(2,0) = 0.005467f;  mag_A_inv(2,1) = -0.109767f; mag_A_inv(2,2) = 1.848900f;

  Serial.println("Keep IMU stationary and flat for calibration...");
  delay(2000);
  calibrateSensors();
  Serial.println("Calibration Done. Starting Filter...");

  // --- Initialize Time ---
  last_time_us = micros();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // --- 1. Time Delta Calculation ---
    unsigned long current_time_us = micros();
    float dt = (current_time_us - last_time_us) / 1000000.0f;
    last_time_us = current_time_us;

    // --- 2. Apply Calibration ---
    // Accel: Raw - Bias
    Vector3f accel_meas(ax, ay, az);
    accel_meas -= accel_bias;

    // Gyro: (Raw - Static Bias) * Units
    Vector3f gyro_meas(gx, gy, gz);
    gyro_meas -= gyro_bias; 
    gyro_meas *= DEG_TO_RADIAN; // Convert to rad/s

    // Mag: A_inv * (Raw - b)  -- using direct mag_A_inv
    Vector3f mag_raw(mx, my, mz); 
    Vector3f mag_meas_temp = mag_A_inv * (mag_raw - mag_b);

    // Remap to Body Frame (NED consistent) -- keep your original remap
    Vector3f mag_meas(-mag_meas_temp.y(), -mag_meas_temp.x(), mag_meas_temp.z());

    // --- 3. Get Measurement Rotation (TRIAD) ---
    Matrix3f R_meas = getTriadRotation(accel_meas, mag_meas);

    // --- If TRIAD-only mode, produce TRIAD quaternion and skip filter ---
    if (OUTPUT_TRIAD_ONLY) {
      Quaternionf q_triad(R_meas);
      q_triad.normalize();
      printQuaternionFlat(q_triad);
      return; // skip complementary filter for this loop iteration
    }

    // --- 4. Direct Complementary Filter on SO(3) ---
    
    // A. Calculate Rotation Error: R_tilde = R_est^T * R_meas
    Matrix3f R_tilde = R_est.transpose() * R_meas;

    // B. Extract Error Vector (omega)
    Matrix3f Pa = 0.5f * (R_tilde - R_tilde.transpose());
    Vector3f omega_err = vex(Pa);

    // C. Update Dynamic Bias Estimate
    b_est += (-kI * omega_err) * dt;

    // D. Correct Gyroscope Reading
    Vector3f omega_corr = gyro_meas - b_est + (kP * omega_err);

    // E. Integrate Rotation
    Matrix3f R_update = rodrigues_exp(omega_corr, dt);
    R_est = R_est * R_update;

    // F. Re-orthogonalize 
    Quaternionf q_final(R_est);
    q_final.normalize();
    R_est = q_final.toRotationMatrix();

    // --- 5. Output ---
    printQuaternionFlat(q_final);
  }
}

// --- UPDATED TRIAD ALGORITHM ---
// Uses a fixed inertial magnetometer reference vector (41.57, 9.23, -32.30)
Matrix3f getTriadRotation(Vector3f accel_meas, Vector3f mag_meas) {
    // 1. Define Inertial Reference Vectors (NED Frame)
    
    // v1: Gravity Reference (Points Down in NED)
    Vector3f v1(0.0f, 0.0f, 1.0f); 
    
    // v2: Fixed magnetic reference vector (user-specified)
    // Vector3f v2(41.57f, 9.23f, -32.30f);
    Vector3f v2(0.0f, -1.0f, 0.0f); 
    v2.normalize();

    // 2. Construct Body Triad (Using Measurements)
    Vector3f w1 = accel_meas.normalized();
    Vector3f w2 = mag_meas.normalized();
    
    Vector3f s1 = w1;
    Vector3f w1xw2 = w1.cross(w2);
    Vector3f s2 = w1xw2.normalized();       
    Vector3f s3 = w1.cross(s2).normalized(); // diffff -- they didnt normalise s3 

    Matrix3f M_body;
    // Columns are s1, s2, s3
    M_body.col(0) = s1;
    M_body.col(1) = s2;
    M_body.col(2) = s3;

    // 3. Construct Inertial Triad (Using References)
    Vector3f r1 = v1;
    Vector3f v1xv2 = v1.cross(v2);
    Vector3f r2 = v1xv2.normalized();
    Vector3f r3 = v1.cross(r2).normalized();

    Matrix3f M_inertial;
    M_inertial.col(0) = r1;
    M_inertial.col(1) = r2;
    M_inertial.col(2) = r3;

    // 4. Calculate Rotation Matrix: R_nb = M_inertial * M_body^T
    return M_inertial * M_body.transpose();
}

// --- Rodrigues' Rotation Formula ---
Matrix3f rodrigues_exp(Vector3f w, float dt) {
    float theta = w.norm() * dt;
    
    if (theta < 1e-6f) {
        return Matrix3f::Identity();
    }

    Vector3f K = w.normalized();
    Matrix3f K_skew;
    K_skew <<  0.0f, -K.z(),  K.y(),
               K.z(),  0.0f, -K.x(),
              -K.y(),  K.x(),  0.0f;

    Matrix3f I = Matrix3f::Identity();
    return I + (sin(theta) * K_skew) + ((1.0f - cos(theta)) * (K_skew * K_skew));
}

// --- Vex Operator ---
Vector3f vex(Matrix3f M) {
    Vector3f v;
    v[0] = M(2, 1); // x
    v[1] = M(0, 2); // y
    v[2] = M(1, 0); // z
    return v;
}

// --- Simple Static Calibration ---
void calibrateSensors() {
    float ax, ay, az, gx, gy, gz;
    const int samples = 500;
    Vector3f a_sum = Vector3f::Zero();
    Vector3f g_sum = Vector3f::Zero();

    for (int i = 0; i < samples; i++) {
        while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable());
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        
        a_sum += Vector3f(ax, ay, az);
        g_sum += Vector3f(gx, gy, gz);
        delay(2);
    }

    accel_bias = a_sum / (float)samples;
    gyro_bias  = g_sum / (float)samples;

    // Target Z = 1.0g (Gravity) for NED Down
    // accel_bias -= Vector3f(0.0f, 0.0f, 1.0f); 

    Serial.print("Gyro Bias: ");
    Serial.print(gyro_bias.x()); Serial.print(", ");
    Serial.print(gyro_bias.y()); Serial.print(", ");
    Serial.println(gyro_bias.z());
}

void printQuaternionFlat(const Quaternionf& q) {
    Serial.print(q.w(), 6); Serial.print(",");
    Serial.print(q.x(), 6); Serial.print(",");
    Serial.print(q.y(), 6); Serial.print(",");
    Serial.println(q.z(), 6);
}