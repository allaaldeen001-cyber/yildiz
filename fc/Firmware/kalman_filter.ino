#include <math.h>

#define timeslice 0.007f
#define var_acc 1.0f

void initKalmanPosVel() {
  current_prob.m11 = 1;
  current_prob.m21 = 0;
  current_prob.m12 = 0;
  current_prob.m22 = 1;
}

void KalmanPosVel() {
  const float Q11 = var_acc * 0.25f * pow(timeslice, 4);
  const float Q12 = var_acc * 0.5f * pow(timeslice, 3);
  const float Q21 = Q12;
  const float Q22 = var_acc * pow(timeslice, 2);
  const float R11 = 0.008f;

  float ps1 = quadprops.height + timeslice * quadprops.kalmanvel_z;
  float ps2 = quadprops.kalmanvel_z;

  float opt = timeslice * current_prob.m22;
  float pp12 = current_prob.m12 + opt + Q12;
  float pp21 = current_prob.m21 + opt;
  float pp11 = current_prob.m11 + timeslice * (current_prob.m12 + pp21) + Q11;
  pp21 += Q21;
  float pp22 = current_prob.m22 + Q22;

  float innovation = quadprops.baro_height - ps1;
  float ic = pp11 + R11;

  float kg1 = pp11 / ic;
  float kg2 = pp21 / ic;

  quadprops.height = ps1 + kg1 * innovation;
  quadprops.kalmanvel_z = ps2 + kg2 * innovation;

  opt = 1 - kg1;
  current_prob.m11 = pp11 * opt;
  current_prob.m12 = pp12 * opt;
  current_prob.m21 = pp21 - pp11 * kg2;
  current_prob.m22 = pp22 - pp12 * kg2;
}
