#include <algorithm>
#include "holonomic.hpp"


Motors holonomic_math(double lx, double ly, double lz, double ax, double ay, double az, int limiter_type){
    Motors m;

    double scale = 1;


    //limiter_type =0 for clamp limiter, 1 for 1/3 limiter and 2 for dynamic limiter
    if (limiter_type == 1){
        scale = 1/3;
    }
    if (limiter_type == 2){
        scale = 1/(abs(lx)+abs(ly)+abs(ax));
    }

    double x = scale * lx;
    double y = scale * ly;
    double r = scale * ax;

    if (limiter_type == 0){
        m.bottom_front_left = std::clamp(y+x+r, (double)-1,(double) 1);
        m.bottom_front_right = std::clamp(y-x-r,(double) -1, (double) 1);
        m.bottom_back_left = std::clamp(y-x+r,(double) -1,(double) 1);
        m.bottom_back_right = std::clamp(y+x-r, (double) -1, (double) 1);
    }
    
    else{
        m.bottom_front_left = y+x+r;
        m.bottom_front_right = y-x-r;
        m.bottom_back_left = y-x+r;
        m.bottom_back_right = y+x-r;
    }

    return m;
}

void test(){
    std::cout << "test";
}

