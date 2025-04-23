#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>

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

inline Tire omni_output(float K_, const Tire &tire_actual_,
                        const Control &control, float speed, float theta = 0) {
  Tire tire_target;
  Tire tire_diff;
  Tire tire_output;

  tire_target.Tire_1 =
      speed * (control.x * std::cos(0 + theta) + control.y * sin(0 + theta)) +
      control.turnspeed;
  tire_target.Tire_2 =
      speed * (control.x * std::cos((std::numbers::pi * 2 / 3) + theta) +
               control.y * std::sin((std::numbers::pi * 2 / 3) + theta)) +
      control.turnspeed;
  tire_target.Tire_3 =
      speed * (control.x * std::cos((std::numbers::pi * 4 / 3) + theta) +
               control.y * std::sin((std::numbers::pi * 4 / 3) + theta)) +
      control.turnspeed;

  tire_diff.Tire_1 = tire_target.Tire_1 - tire_actual_.Tire_1;
  tire_diff.Tire_2 = tire_target.Tire_2 - tire_actual_.Tire_2;
  tire_diff.Tire_3 = tire_target.Tire_3 - tire_actual_.Tire_3;

  tire_output.Tire_1 = std::clamp(tire_diff.Tire_1 * K_, -10000.0f, 10000.0f);
  tire_output.Tire_2 = std::clamp(tire_diff.Tire_2 * K_, -10000.0f, 10000.0f);
  tire_output.Tire_3 = std::clamp(tire_diff.Tire_3 * K_, -10000.0f, 10000.0f);

  return tire_output;
}
