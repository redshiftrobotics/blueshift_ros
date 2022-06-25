struct Motors
{
    double top_front_left;
    double bottom_front_left;
    double top_front_right;
    double bottom_front_right;
    double top_back_left;
    double bottom_back_left;
    double top_back_right;
    double bottom_back_right;
};

Motors holonomic_math(double lx, double ly, double lz, double ax, double ay, double az, int limiter_type);