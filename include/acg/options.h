#pragma once

#include <acg/macro.h>
#include <acg/types.h>

namespace acg {
struct LinkOptions {
  LinkOptions(Width width, Length length) : link_width_(width.get()), link_length_(length.get()) {}

  ACG_ARG(float, link_width);
  ACG_ARG(float, link_length);
  ACG_ARG(float, density) = 1.f;
};

struct JointOptions {
  JointOptions() = default;

  ACG_ARG(float, upper_angle) = 0.f;
  ACG_ARG(float, lower_angle) = 0.f;
  ACG_ARG(bool, enable_limit) = false;
  ACG_ARG(float, motor_speed) = 0.f;
  ACG_ARG(bool, enable_motor) = false;
  ACG_ARG(float, max_motor_torque) = 10000.f;
};
}  // namespace acg
