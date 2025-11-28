/*
 * Kalman Filter for Position and Velocity Estimation
 * Fuses barometer data for smoother altitude control
 */

#define TIMESLICE 0.007  // 140 Hz
#define VAR_ACC 1.0

void initKalmanPosVel(void) {
  current_prob.m11 = 1;
  current_prob.m21 = 0;
  current_prob.m12 = 0;
  current_prob.m22 = 1;
  
  quadprops.height = 0;
  quadprops.kalmanvel_z = 0;
  quadprops.baro_height = 0;
}

void KalmanPosVel() {
  // Process noise covariance matrix Q
  const float Q11 = VAR_ACC * 0.25 * (TIMESLICE * TIMESLICE * TIMESLICE * TIMESLICE);
  const float Q12 = VAR_ACC * 0.5 * (TIMESLICE * TIMESLICE * TIMESLICE);
  const float Q21 = VAR_ACC * 0.5 * (TIMESLICE * TIMESLICE * TIMESLICE);
  const float Q22 = VAR_ACC * (TIMESLICE * TIMESLICE);
  
  // Measurement noise covariance R
  const float R11 = 0.008;
  
  float ps1, ps2, opt;
  float pp11, pp12, pp21, pp22;
  float inn, ic, kg1, kg2;
  
  // Prediction step
  ps1 = quadprops.height + TIMESLICE * quadprops.kalmanvel_z;
  ps2 = quadprops.kalmanvel_z;
  
  opt = TIMESLICE * current_prob.m22;
  pp12 = current_prob.m12 + opt + Q12;
  pp21 = current_prob.m21 + opt;
  pp11 = current_prob.m11 + TIMESLICE * (current_prob.m12 + pp21) + Q11;
  pp21 += Q21;
  pp22 = current_prob.m22 + Q22;
  
  // Update step
  inn = quadprops.baro_height - ps1;  // Innovation
  ic = pp11 + R11;  // Innovation covariance
  
  // Kalman gains
  kg1 = pp11 / ic;
  kg2 = pp21 / ic;
  
  // State update
  quadprops.height = ps1 + kg1 * inn;
  quadprops.kalmanvel_z = ps2 + kg2 * inn;
  
  // Covariance update
  opt = 1 - kg1;
  current_prob.m11 = pp11 * opt;
  current_prob.m12 = pp12 * opt;
  current_prob.m21 = pp21 - pp11 * kg2;
  current_prob.m22 = pp22 - pp12 * kg2;
}
