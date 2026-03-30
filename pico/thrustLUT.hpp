#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static const float DS_MIN = 48;
static const float DS_MAX = 2047;

static const float DEADZONE = 0.3f;

static const float NEUTRAL = 48.0f;

static float smooth(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x * x * (3.0f - 2.0f * x);
}

static float line(float m, float b, float x) {
  return m * x + b;
}

static float negativeLUT(float t) {
  if (t < -35.245375f) return line(-24.063208f, 1083.193100f, t);
  if (t < -27.950000f) return line(-15.436931f, 1394.560894f, t);
  if (t < -22.065000f) return line(-19.105131f, 1289.319627f, t);
  if (t < -16.359000f) return line(-19.369792f, 1282.107987f, t);
  if (t < -11.230000f) return line(-22.324623f, 1233.694270f, t);
  if (t < -7.090750f) return line(-26.527299f, 1183.575054f, t);
  if (t < -3.429125f) return line(-30.847016f, 1152.325604f, t);
  return line(-39.222849f, 1126.814094f, t);
}

static float positiveLUT(float t) {
  if (t < 3.912000f) return line(32.124395f, 125.585490f, t);
  if (t < 8.713000f) return line(23.893044f, 155.949785f, t);
  if (t < 14.092000f) return line(21.070029f, 181.438211f, t);
  if (t < 20.426000f) return line(18.156720f, 223.273723f, t);
  if (t < 27.827000f) return line(15.094079f, 286.095627f, t);
  if (t < 35.718000f) return line(15.005882f, 290.617719f, t);
  if (t < 44.364000f) return line(12.503613f, 378.937966f, t);
  return line(15.162766f, 257.577506f, t);
}

float thrustToDshot(float thrust) {

  // Deadzone
  if (fabs(thrust) < DEADZONE) {
    return NEUTRAL;
  }

  float dshot;

  if (thrust > 0.0f) {
    dshot = positiveLUT(thrust);
  }
  else {
    dshot = negativeLUT(thrust);
  }
  // Clamp
  if (dshot < DS_MIN) dshot = DS_MIN;
  if (dshot > DS_MAX) dshot = DS_MAX;

  return dshot;
}

// int main() {
//   for (float i = -3.0f; i < 3.0f; i += 0.05) {
//     printf("%.2f, %.2f\n", i, thrustToDshot(i));
//   }
//   return 0;
// }