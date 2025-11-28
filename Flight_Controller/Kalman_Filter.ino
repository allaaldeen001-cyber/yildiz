/*
 * KALMAN FILTER - Position and Velocity Estimation
 * Provides smooth altitude estimation from noisy barometer data
 */

//===================== INITIALIZE KALMAN FILTER =====================
void initKalmanPosVel(void) {
  current_prob.m11 = 1.0;
  current_prob.m21 = 0.0;
  current_prob.m12 = 0.0;
  current_prob.m22 = 1.0;
  
  quadprops.height = 0.0;
  quadprops.kalmanvel_z = 0.0;
  quadprops.baro_height = 0.0;
}

//===================== KALMAN FILTER UPDATE =====================
#define timeslice 0.007  // 140 Hz loop rate
#define var_acc 1        // Process noise variance

void KalmanPosVel() {
  // Process noise covariance matrix Q
  const float Q11 = var_acc * 0.25 * (timeslice * timeslice * timeslice * timeslice);
  const float Q12 = var_acc * 0.5 * (timeslice * timeslice * timeslice);
  const float Q21 = var_acc * 0.5 * (timeslice * timeslice * timeslice);
  const float Q22 = var_acc * (timeslice * timeslice);
  
  // Measurement noise covariance
  const float R11 = 0.008;
  
  float ps1, ps2, opt;
  float pp11, pp12, pp21, pp22;
  float inn, ic, kg1, kg2;
  
  // Prediction step
  ps1 = quadprops.height + timeslice * quadprops.kalmanvel_z;
  ps2 = quadprops.kalmanvel_z;
  
  // Predicted covariance
  opt = timeslice * current_prob.m22;
  pp12 = current_prob.m12 + opt + Q12;
  pp21 = current_prob.m21 + opt;
  pp11 = current_prob.m11 + timeslice * (current_prob.m12 + pp21) + Q11;
  pp21 += Q21;
  pp22 = current_prob.m22 + Q22;
  
  // Innovation (measurement residual)
  inn = quadprops.baro_height - ps1;
  ic = pp11 + R11;
  
  // Kalman gain
  kg1 = pp11 / ic;
  kg2 = pp21 / ic;
  
  // Update estimate
  quadprops.height = ps1 + kg1 * inn;
  quadprops.kalmanvel_z = ps2 + kg2 * inn;
  
  // Update covariance
  opt = 1.0 - kg1;
  current_prob.m11 = pp11 * opt;
  current_prob.m12 = pp12 * opt;
  current_prob.m21 = pp21 - pp11 * kg2;
  current_prob.m22 = pp22 - pp12 * kg2;
}
