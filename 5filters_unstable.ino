#include <Arduino_LSM9DS1.h>
#include "eigen.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

// --- Tuning Parameters ---
const float kP = 0.05f;   // Proportional gain
const float kI = 0.05f;   // Integral gain
const float DEG_TO_RADIAN = M_PI / 180.0f;

// --- Calibration Globals (User Provided) ---
Vector3f accel_bias = Vector3f::Zero();
Vector3f gyro_bias  = Vector3f::Zero();
Vector3f mag_b(27.250990f, 41.956763f, -17.595884f);
Matrix3f mag_A_inv; // Set in setup

// --- Filter States for 5 Estimators ---
Matrix3f R_gyro = Matrix3f::Identity();
Matrix3f R_triad = Matrix3f::Identity();
Matrix3f R_direct = Matrix3f::Identity();
Matrix3f R_passive = Matrix3f::Identity();
Matrix3f R_explicit = Matrix3f::Identity();

// Bias estimates for the filters
Vector3f b_direct = Vector3f::Zero();
Vector3f b_passive = Vector3f::Zero();
Vector3f b_explicit = Vector3f::Zero();

unsigned long last_time_us = 0;

// --- Function Declarations ---
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
  
  // --- User Provided Mag Matrix ---
  mag_A_inv.setZero();
  mag_A_inv(0,0) = 1.888865f;  mag_A_inv(0,1) = 0.049203f;  mag_A_inv(0,2) = 0.005467f;
  mag_A_inv(1,0) = 0.049203f;  mag_A_inv(1,1) = 1.830249f;  mag_A_inv(1,2) = -0.109767f;
  mag_A_inv(2,0) = 0.005467f;  mag_A_inv(2,1) = -0.109767f; mag_A_inv(2,2) = 1.848900f;

  Serial.println("Keep IMU stationary and flat for calibration...");
  calibrateSensors();
  Serial.println("Calibration Done. Starting 5-Way Filter...");

  // Initialize Time
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

    // --- 1. Time Delta ---
    unsigned long current_time_us = micros();
    float dt = (current_time_us - last_time_us) / 1000000.0f;
    last_time_us = current_time_us;

    // --- 2. Apply Calibration (User Logic) ---
    Vector3f accel_meas(ax, ay, az);
    accel_meas -= accel_bias; // User logic: raw - bias

    Vector3f gyro_meas(gx, gy, gz);
    gyro_meas -= gyro_bias; 
    gyro_meas *= DEG_TO_RADIAN;

    Vector3f mag_raw(mx, my, mz); 
    Vector3f mag_meas_temp = mag_A_inv * (mag_raw - mag_b);
    // User Remap
    Vector3f mag_meas(-mag_meas_temp.y(), -mag_meas_temp.x(), mag_meas_temp.z());

    // =========================================================
    // ESTIMATOR 1: Pure Gyro (Drifts over time)
    // =========================================================
    R_gyro = R_gyro * rodrigues_exp(gyro_meas, dt);

    // =========================================================
    // ESTIMATOR 2: TRIAD (Noisy, but no drift)
    // =========================================================
    // Using your specific getTriadRotation function
    R_triad = getTriadRotation(accel_meas, mag_meas);

    // =========================================================
    // ESTIMATOR 3: Direct Complementary (SO3)
    // Strategy: Integrate Gyro -> Calculate Error vs TRIAD -> Correct R directly
    // =========================================================
    // Predict
    Matrix3f R_direct_pred = R_direct * rodrigues_exp(gyro_meas - b_direct, dt);
    
    // Error (R_pred^T * R_meas)
    Matrix3f R_tilde_dir = R_direct_pred.transpose() * R_triad;
    Vector3f w_err_dir = vex(0.5f * (R_tilde_dir - R_tilde_dir.transpose()));
    
    // Bias Update
    b_direct += -kI * w_err_dir * dt;

    // Direct Correction: R = R_pred * exp(kP * error)
    // We apply the correction rotation "on top" of the prediction
    R_direct = R_direct_pred * rodrigues_exp(kP * w_err_dir, dt);

    // =========================================================
    // ESTIMATOR 4: Passive Complementary (SO3)
    // Strategy: Calculate Error vs TRIAD -> Add to Rate -> Integrate
    // (This matches your provided code structure)
    // =========================================================
    Matrix3f R_tilde_pass = R_passive.transpose() * R_triad;
    Vector3f w_err_pass = vex(0.5f * (R_tilde_pass - R_tilde_pass.transpose()));

    b_passive += (-kI * w_err_pass) * dt;
    Vector3f w_corr_pass = gyro_meas - b_passive + (kP * w_err_pass);
    
    R_passive = R_passive * rodrigues_exp(w_corr_pass, dt);

    // =========================================================
    // ESTIMATOR 5: Explicit Complementary (Vector / Mahony)
    // Strategy: Cross product of estimated gravity/mag vs measured
    // =========================================================
    // Inertial references from your TRIAD function: v1(0,0,1), v2(0,-1,0)
    Vector3f g_ref(0,0,1); 
    Vector3f m_ref(0,-1,0); 

    // Map inertial refs to body frame using current estimate
    Vector3f v_g_est = R_explicit.transpose() * g_ref;
    Vector3f v_m_est = R_explicit.transpose() * m_ref;

    // Error is sum of cross products
    Vector3f w_err_expl = accel_meas.normalized().cross(v_g_est) + 
                          mag_meas.normalized().cross(v_m_est);
    
    b_explicit += -kI * w_err_expl * dt;
    Vector3f w_corr_expl = gyro_meas - b_explicit + (kP * w_err_expl);

    R_explicit = R_explicit * rodrigues_exp(w_corr_expl, dt);

    // =========================================================
    // Normalization & Output
    // =========================================================
    
    // Helper lambda to process and print
    auto processAndPrint = [](Matrix3f &R) {
        Quaternionf q(R);
        q.normalize();
        R = q.toRotationMatrix(); // Re-orthogonalize internal state
        Serial.print(q.w(), 4); Serial.print(",");
        Serial.print(q.x(), 4); Serial.print(",");
        Serial.print(q.y(), 4); Serial.print(",");
        Serial.print(q.z(), 4); Serial.print(",");
    };

    processAndPrint(R_gyro);
    processAndPrint(R_triad);
    processAndPrint(R_direct);
    processAndPrint(R_passive);
    
    // For the last one, we need a newline at the end
    Quaternionf q_exp(R_explicit);
    q_exp.normalize();
    R_explicit = q_exp.toRotationMatrix();
    Serial.print(q_exp.w(), 4); Serial.print(",");
    Serial.print(q_exp.x(), 4); Serial.print(",");
    Serial.print(q_exp.y(), 4); Serial.print(",");
    Serial.println(q_exp.z(), 4);
  }
}

// --- YOUR TRIAD FUNCTION (UNCHANGED) ---
Matrix3f getTriadRotation(Vector3f accel_meas, Vector3f mag_meas) {
    Vector3f v1(0.0f, 0.0f, 1.0f); 
    Vector3f v2(0.0f, -1.0f, 0.0f); 
    v2.normalize();

    Vector3f w1 = accel_meas.normalized();
    Vector3f w2 = mag_meas.normalized();
    
    Vector3f s1 = w1;
    Vector3f w1xw2 = w1.cross(w2);
    Vector3f s2 = w1xw2.normalized();        
    Vector3f s3 = w1.cross(s2).normalized(); 

    Matrix3f M_body;
    M_body.col(0) = s1; M_body.col(1) = s2; M_body.col(2) = s3;

    Vector3f r1 = v1;
    Vector3f v1xv2 = v1.cross(v2);
    Vector3f r2 = v1xv2.normalized();
    Vector3f r3 = v1.cross(r2).normalized();

    Matrix3f M_inertial;
    M_inertial.col(0) = r1; M_inertial.col(1) = r2; M_inertial.col(2) = r3;

    return M_inertial * M_body.transpose();
}

// --- HELPERS (UNCHANGED) ---
Matrix3f rodrigues_exp(Vector3f w, float dt) {
    float theta = w.norm() * dt;
    if (theta < 1e-6f) return Matrix3f::Identity();

    Vector3f K = w.normalized();
    Matrix3f K_skew;
    K_skew <<  0.0f, -K.z(),  K.y(),
               K.z(),  0.0f, -K.x(),
              -K.y(),  K.x(),  0.0f;

    Matrix3f I = Matrix3f::Identity();
    return I + (sin(theta) * K_skew) + ((1.0f - cos(theta)) * (K_skew * K_skew));
}

Vector3f vex(Matrix3f M) {
    return Vector3f(M(2, 1), M(0, 2), M(1, 0));
}

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
}