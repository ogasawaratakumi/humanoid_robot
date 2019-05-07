//初期状態に応じてグラフが変化

#define _USE_MAT_DEFINES
#include <iostream>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
    double g = 9.8;
    double z = 0.8;
    double Tc = sqrt(z/g);
    double x0[4] = {-0.151, -0.2, 0.2, 0.151};
    double v0[4] = {0.467, 0.791, -0.791, -0.467};
    int cnt_max = 800;
    double time = 0.0;
    std::vector<double> x1(cnt_max), y1(cnt_max), y2(cnt_max), y3(cnt_max), y4(cnt_max);

    for(int cnt_now=0; time<=0.001*cnt_max; cnt_now+=1){
            x1[cnt_now] = time;
            y1[cnt_now] = (x0[0]*cosh(time/Tc))+Tc*v0[0]*sinh(time/Tc);
            y2[cnt_now] = (x0[1]*cosh(time/Tc))+Tc*v0[1]*sinh(time/Tc);
            y3[cnt_now] = (x0[2]*cosh(time/Tc))+Tc*v0[2]*sinh(time/Tc);
            y4[cnt_now] = (x0[3]*cosh(time/Tc))+Tc*v0[3]*sinh(time/Tc);
            time+=0.001;
    }

    plt::subplot(2, 2, 1);
    plt::plot(x1, y1);
    plt::xlim(0.0, 0.8);
    plt::ylim(-0.2, 0.2);

    plt::subplot(2, 2, 2);
    plt::plot(x1, y2);
    plt::xlim(0.0, 0.8);
    plt::ylim(-0.2, 0.2);

    plt::subplot(2, 2, 3);
    plt::plot(x1, y3);
    plt::xlim(0.0, 0.8);
    plt::ylim(-0.2, 0.2);

    plt::subplot(2, 2, 4);
    plt::plot(x1, y4);
    plt::xlim(0.0, 0.8);
    plt::ylim(-0.2, 0.2);
   
    plt::show();
}

