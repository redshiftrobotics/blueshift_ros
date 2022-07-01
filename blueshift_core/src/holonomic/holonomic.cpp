#include <algorithm>
#include <math.h>
#include "holonomic.hpp"
#include "blueshift_interfaces/msg/motors.hpp"


blueshift_interfaces::msg::Motors holonomic_math(double lx, double ly, double lz, double ax, double ay, double az, int limiter_type)
{
    auto m = blueshift_interfaces::msg::Motors();

    double bottom_scale = (double) 1;
    double top_scale = (double) 1;

    // limiter_type = 0 for clamp limiter, 1 for 1/3 limiter and 2 for dynamic limiter
    if (limiter_type == 1)
    {
        bottom_scale = (double) 1 / 3;
        top_scale = (double) 1 / 3;
    }
    if (limiter_type == 2)
    {
        if (lx!= (double) 0 || ly!= (double) 0 || az!= (double) 0){
            bottom_scale = (double) (1 / (fabs(lx) + fabs(ly) + fabs(az)));
        }
        
        if (lz!=0 || ax!=0 || ay!=0){
            top_scale = (double) (1 / (fabs(lz) + fabs(ax) + fabs(ay)));
        }
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
