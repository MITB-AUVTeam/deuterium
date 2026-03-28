static const float DS_MIN = 1100;
static const float DS_MAX = 1900;

static const float DEADZONE = 300.0;
static const float SOFTZONE = 100.0;

static const float NEUTRAL = 1118.0;

static float smooth(float x) {
  if (x < 0.0) return 0.0;
  if (x > 1.0) return 1.0;
  return x * x * (3 - 2 * x);
}

static float line(float m, float b, float x) {
  return m * x + b;
}

static float negativeLUT(float t) {
  if (t < 100) return line(-0.023303, 1112.522106, t);
  if (t < 200) return line(-0.015108, 1405.382453, t);
  if (t < 300) return line(-0.017990, 1319.950737, t);
  if (t < 400) return line(-0.019187, 1286.576244, t);
  if (t < 500) return line(-0.019336, 1282.509349, t);
  if (t < 600) return line(-0.023092, 1223.174285, t);
  if (t < 700) return line(-0.027684, 1172.797434, t);
  return line(-0.035601, 1132.755869, t);
}

static float positiveLUT(float t) {
  if (t < 100) return line(0.028581, 132.168533, t);
  if (t < 200) return line(0.021986, 171.323136, t);
  if (t < 300) return line(0.018402, 218.989333, t);
  if (t < 400) return line(0.015280, 281.793738, t);
  if (t < 500) return line(0.015789, 267.183323, t);
  if (t < 600) return line(0.013614, 337.550535, t);
  if (t < 700) return line(0.012093, 396.288025, t);
  return line(0.015859, 222.987404, t);
}

float thrustToDshot(float thrust) {

  float absT = abs(thrust / 1000.f);

  if (absT < DEADZONE) {
    return NEUTRAL;
  }

  float dshot;

  if (thrust > 0) {
    dshot = positiveLUT(absT);
  }
  else {
    dshot = negativeLUT(absT);
  }

  if (absT < DEADZONE + SOFTZONE) {
    float x = (absT - DEADZONE) / SOFTZONE;
    float w = smooth(x);
    dshot = NEUTRAL * (1 - w) + dshot * w;
  }

  if (dshot < DS_MIN) dshot = DS_MIN;
  if (dshot > DS_MAX) dshot = DS_MAX;

  return dshot;
}