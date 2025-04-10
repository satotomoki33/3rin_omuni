#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

struct Control {
  float x = 0;
  float y = 0;
  float turnspeed = 0;
};

struct Tire {
  float Tire_1 = 0;
  float Tire_2 = 0;
  float Tire_3 = 0;
};

class omuni3 {

public:
  omuni3() {}

  void get_speed(Tire tire_actual) {
    tire_actual_.Tire_1 = tire_actual.Tire_1;
    tire_actual_.Tire_2 = tire_actual.Tire_2;
    tire_actual_.Tire_3 = tire_actual.Tire_3;
  }
  Tire output(Control control, float speed) {
    Tire tire_target;
    Tire tire_diff;
    Tire tire_output;

    // X, Yを300度回転
    //float theta = M_PI * 5 / 3; // 300度
     float theta = 0; // 0度
    float rotated_x = control.x * cos(theta) - control.y * sin(theta);
    float rotated_y = control.x * sin(theta) + control.y * cos(theta);

    tire_target.Tire_1 =
        speed * (rotated_x * cos(0) + rotated_y * sin(0)) + control.turnspeed;
    tire_target.Tire_2 = speed * (rotated_x * cos((M_PI * 2 / 3)) +
                                  rotated_y * sin((M_PI * 2 / 3))) +
                         control.turnspeed;
    tire_target.Tire_3 = speed * (rotated_x * cos((M_PI * 4 / 3)) +
                                  rotated_y * sin((M_PI * 4 / 3))) +
                         control.turnspeed;

    tire_diff.Tire_1 = tire_target.Tire_1 - tire_actual_.Tire_1;
    tire_diff.Tire_2 = tire_target.Tire_2 - tire_actual_.Tire_2;
    tire_diff.Tire_3 = tire_target.Tire_3 - tire_actual_.Tire_3;

    tire_output.Tire_1 = clamp(tire_diff.Tire_1 * K_, -10000, 10000);
    tire_output.Tire_2 = clamp(tire_diff.Tire_2 * K_, -10000, 10000);
    tire_output.Tire_3 = clamp(tire_diff.Tire_3 * K_, -10000, 10000);

    return tire_output;
  }

private:
  Tire tire_actual_;
  float K_ = 100;
  int clamp(float val, int lo, int hi) {
    if (val < lo)
      return lo;
    if (val > hi)
      return hi;
    return static_cast<int>(val);
  }
};
