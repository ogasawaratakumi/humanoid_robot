//歩行素片　終端速度について
#include <iostream>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
    double g = 9.8;
    double z = 0.8;
    double x = 0.06;
    double y = -0.03;
    double Tc = sqrt(z/g);
    double Ts = 0.5;
    double C = cosh(Ts/Tc);
    double S = sinh(Ts/Tc);
    double vx = (-x*(C-1))/(Tc*S);  //x方向の重心速度
    double vy = (-y*(C+1))/(Tc*S);  //y方向の重心速度
    double time = 0.0;
    int cnt_max = 500;
    std::vector<double> x1(cnt_max), y1(cnt_max), y2(cnt_max);

    for(int cnt_now=0; time<=0.001*cnt_max; cnt_now+=1){
        x1[cnt_now] = time;
        y1[cnt_now] = (x*cosh(time/Tc))+(Tc*vx*sinh(time/Tc));
        y2[cnt_now] = (y*cosh(time/Tc))+(Tc*vy*sinh(time/Tc));
        time += 0.001;
    }

   // std::cout << Tc << std::endl;
    plt::subplot(2, 1, 1);
    plt::plot(x1, y1);
    plt::ylim(0.0, 0.06);

    plt::subplot(2, 1, 2);
    plt::plot(x1, y2);

    plt::show();
}
