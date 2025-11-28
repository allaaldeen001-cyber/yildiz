/*
 * KalmanFilter.ino
 * ================
 * Kalman filter for altitude/velocity estimation
 * Fuses barometer data to estimate height and vertical velocity
 * This file is part of the FlightController project
 */

// Kalman filter constants
#define KALMAN_TIMESLICE 0.007143  // ~1/140 Hz
#define KALMAN_VAR_ACC 1.0         // Process noise (acceleration variance)

// Measurement noise (pressure sensor noise)
#define KALMAN_R11 0.008

void initKalmanPosVel() {
    // Initialize probability matrix (identity)
    currentProb.m11 = 1.0;
    currentProb.m12 = 0.0;
    currentProb.m21 = 0.0;
    currentProb.m22 = 1.0;
    
    // Initialize state
    quadProps.height = 0.0;
    quadProps.kalmanVelZ = 0.0;
    quadProps.baroHeight = 0.0;
}

void KalmanPosVel() {
    // Process noise covariance matrix Q
    // Based on constant acceleration model
    float dt = KALMAN_TIMESLICE;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    
    float Q11 = KALMAN_VAR_ACC * 0.25 * dt4;
    float Q12 = KALMAN_VAR_ACC * 0.5 * dt3;
    float Q21 = KALMAN_VAR_ACC * 0.5 * dt3;
    float Q22 = KALMAN_VAR_ACC * dt2;
    
    // Measurement noise
    float R11 = KALMAN_R11;
    
    // === Prediction Step ===
    
    // Predicted state: x_pred = F * x
    // F = [1, dt; 0, 1]
    float ps1 = quadProps.height + dt * quadProps.kalmanVelZ;  // Predicted position
    float ps2 = quadProps.kalmanVelZ;                          // Predicted velocity
    
    // Predicted covariance: P_pred = F * P * F' + Q
    float opt = dt * currentProb.m22;
    float pp12 = currentProb.m12 + opt + Q12;
    float pp21 = currentProb.m21 + opt;
    float pp11 = currentProb.m11 + dt * (currentProb.m12 + pp21) + Q11;
    pp21 += Q21;
    float pp22 = currentProb.m22 + Q22;
    
    // === Update Step ===
    
    // Innovation: y = z - H * x_pred
    // H = [1, 0] (we only measure position)
    float inn = quadProps.baroHeight - ps1;
    
    // Innovation covariance: S = H * P_pred * H' + R
    float ic = pp11 + R11;
    
    // Kalman gain: K = P_pred * H' * S^-1
    float kg1 = pp11 / ic;
    float kg2 = pp21 / ic;
    
    // Updated state: x = x_pred + K * y
    quadProps.height = ps1 + kg1 * inn;
    quadProps.kalmanVelZ = ps2 + kg2 * inn;
    
    // Updated covariance: P = (I - K * H) * P_pred
    opt = 1.0 - kg1;
    currentProb.m11 = pp11 * opt;
    currentProb.m12 = pp12 * opt;
    currentProb.m21 = pp21 - pp11 * kg2;
    currentProb.m22 = pp22 - pp12 * kg2;
}

// Optional: Reset Kalman filter to current barometer reading
void resetKalmanFilter() {
    quadProps.height = quadProps.baroHeight;
    quadProps.kalmanVelZ = 0.0;
    initKalmanPosVel();
}

// Get filtered height
float getKalmanHeight() {
    return quadProps.height;
}

// Get filtered vertical velocity
float getKalmanVelocityZ() {
    return quadProps.kalmanVelZ;
}
