#pragma once
#include "blueshift_interfaces/msg/motors.hpp"

blueshift_interfaces::msg::Motors holonomic_math(double lx, double ly, double lz, double ax, double ay, double az, int limiter_type);