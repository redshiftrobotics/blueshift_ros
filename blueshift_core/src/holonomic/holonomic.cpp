#include <algorithm>
#include "holonomic.hpp"
#include "blueshift_interfaces/msg/motors.hpp"


blueshift_interfaces::msg::Motors holonomic_math(double lx, double ly, double lz, double ax, double ay, double az, int limiter_type)
{
    auto m = blueshift_interfaces::msg::Motors();

    double bottom_scale = 1;
    double top_scale = 1;

    // limiter_type = 0 for clamp limiter, 1 for 1/3 limiter and 2 for dynamic limiter
    if (limiter_type == 1)
    {
        bottom_scale = 1 / 3;
        top_scale = 1 / 3;
    }
    if (limiter_type == 2)
    {
        bottom_scale = 1 / (abs(lx) + abs(ly) + abs(az));
        top_scale = 1 / (abs(lz) + abs(ax) + abs(ay));
    }

    double x = bottom_scale * lx;
    double y = bottom_scale * ly;
    double yaw = bottom_scale * az;

    double z = top_scale * lz;
    double roll = top_scale * ax;
    double pitch = top_scale * ay;

    if (limiter_type == 0)
    {
        m.top_front_left = std::clamp(z - roll - pitch, (double)-1, (double)1);
        m.top_front_right = std::clamp(z + roll - pitch, (double)-1, (double)1);
        m.top_back_left = std::clamp(z - roll + pitch, (double)-1, (double)1);
        m.top_back_right = std::clamp(z + roll + pitch, (double)-1, (double)1);
        m.bottom_front_left = std::clamp(y + x + yaw, (double)-1, (double)1);
        m.bottom_front_right = std::clamp(y - x - yaw, (double)-1, (double)1);
        m.bottom_back_left = std::clamp(y - x + yaw, (double)-1, (double)1);
        m.bottom_back_right = std::clamp(y + x - yaw, (double)-1, (double)1);
    }

    else
    {
        m.top_front_left = z - roll - pitch;
        m.top_front_right = z + roll - pitch;
        m.top_back_left = z - roll + pitch;
        m.top_back_right = z + roll + pitch;
        m.bottom_front_left = y + x + yaw;
        m.bottom_front_right = y - x - yaw;
        m.bottom_back_left = y - x + yaw;
        m.bottom_back_right = y + x - yaw;
    }

    return m;
}
