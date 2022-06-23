struct Motors{
    double top_front_left=0;
    double bottom_front_left=0;
    double top_front_right=0;
    double bottom_front_right=0;
    double top_back_left=0;
    double bottom_back_left=0;
    double top_back_right=0;
    double bottom_back_right=0;
};

Motors holonomic_math(double lx,double ly,double lz, double ax, double ay, double az){
    Motors m;
    
    double scale = 1/(lx+ly+ax);

    double x = scale * lx;
    double y = scale * ly;
    double r = scale * ax;
    
    m.bottom_front_left = y+x+r;
    m.bottom_front_right = y-x-r;
    m.bottom_back_left = y-x+r;
    m.bottom_back_right = y+x-r;

    return m;
}

