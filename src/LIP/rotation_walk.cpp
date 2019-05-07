#include <iostream>
#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include "matplotlibcpp.h"

using namespace std;
using namespace Eigen;

namespace plt = matplotlibcpp;

static const float Zc = 0.8;
static const float g = 9.81;

class LIP {
    private:
        float t_sup;
        float dt;
        float Tc;
        float C,S,D;
        int count;
        const int a,b;
        float x,y;                            //重心位置
        float xi,yi;                          //n歩目の重心位置
        float xb,yb;                          //n歩目で用いる歩行素片パラメータ
        float xd,yd;                          //目標状態
        float dx,dy;                          //重心速度
        float dxi,dyi;                        //n歩目の重心速度
        float dxb,dyb;                        //目標状態
        float vxb,vyb;                        //終端速度
        float px,py;
        float pxa,pya;
    public:
        float T;
        vector<Vector3f> foot_step_list;
        vector<float>x_list, y_list;
        vector<float>px_list, py_list;
        vector<float>pxa_list, pya_list;
        LIP(float _t_sup, float _dt, int _a, int _b) : t_sup(_t_sup), dt(_dt), a(_a), b(_b) {
            Tc = sqrt(Zc/g);
            D = a*pow((cosh(t_sup/Tc)-1),2) + b*pow((sinh(t_sup/Tc)/Tc),2);
            S = sinh(Zc/Tc);
            C = cosh(Zc/Tc);
            T = 0;

            //step 1
            x = 0.0; y = 0.01;
            dx = 0; dy = 0;
            xi = x; yi = y;
            dxi = dx; dyi = dy;
            px = 0.0; py = 0.0;
            pxa = px; pya = py;
        }

        void step2();
        void step3_and_4(int count);
        void step5();
        void step6();
        void step7();
        void step8();
        void graph_show();
};

float conversion(float degree) {
    float radian = M_PI*degree/180.0f;
    return radian;
}

void LIP::step2() {
    this->foot_step_list.push_back(Vector3f(0.0,0.2,0.0));
    this->foot_step_list.push_back(Vector3f(0.25,0.2,20.0));
    this->foot_step_list.push_back(Vector3f(0.25,0.2,40.0));
    this->foot_step_list.push_back(Vector3f(0.25,0.2,60.0));
    this->foot_step_list.push_back(Vector3f(0.0,0.2,60.0));
    this->foot_step_list.push_back(Vector3f(0.0,0.0,0.0));
}

void LIP::step3_and_4(int count) {
    float x, y;
    float dx, dy;

    this->count = count;

    for(float t=0.0; t<=t_sup;t+=dt) {
        x = (xi-pxa)*cosh(t/Tc) + Tc*dxi*sinh(t/Tc) + pxa;
        y = (yi-pya)*cosh(t/Tc) + Tc*dyi*sinh(t/Tc) + pya;
        dx = (xi-pxa)/Tc*sinh(t/Tc) + dxi*cosh(t/Tc);
        dy = (yi-pya)/Tc*sinh(t/Tc) + dyi*cosh(t/Tc);
        x_list.push_back(x); y_list.push_back(y);
    }

    T =+ t_sup;
    xi = x; yi = y;
    dxi = dx; dyi = dy;
}

void LIP::step5() {
    //px = px + foot_step_list[count][0];
    //py = py - (pow(-1,count+1)*foot_step_list[count][1]);
    px = px_list[count]+cos(conversion(foot_step_list[count][2]))*foot_step_list[count][0] - sin(conversion(foot_step_list[count][2]))*(-1*pow(-1,count+1)*foot_step_list[count][1]);
    py = py_list[count]+sin(conversion(foot_step_list[count][2]))*foot_step_list[count][0] + cos(conversion(foot_step_list[count][2]))*(-1*pow(-1,count+1)*foot_step_list[count][1]);
    px_list.push_back(px); py_list.push_back(py);
}

void LIP::step6() {
    //xb = foot_step_list[count+1][0]/2;
    //yb = pow(-1,count+1)*foot_step_list[count+1][1]/2;
    xb = cos(conversion(foot_step_list[count+1][2]))*foot_step_list[count+1][0]/2 - sin(conversion(foot_step_list[count+1][2]))*pow(-1,count+1)*foot_step_list[count+1][1]/2;
    yb = sin(conversion(foot_step_list[count+1][2]))*foot_step_list[count+1][0]/2 + cos(conversion(foot_step_list[count+1][2]))*pow(-1,count+1)*foot_step_list[count+1][1]/2;
    //vxb = ((cosh(t_sup/Tc)+1)/(Tc*sinh(t_sup/Tc)))*xb;
    //vyb = ((cosh(t_sup/Tc)-1)/(Tc*sinh(t_sup/Tc)))*yb;
    vxb = cos(conversion(foot_step_list[count+1][2]))*(1+C)/(Tc*S)*xb -sin(conversion(foot_step_list[count+1][2]))*(C-1)/(Tc*S)*yb;
    vyb = sin(conversion(foot_step_list[count+1][2]))*(1+C)/(Tc*S)*xb + cos(conversion(foot_step_list[count+1][2]))*(C-1)/(Tc*S)*yb;
}

void LIP::step7() {
    xd = px+xb;
    dxb = vxb;
    yd = py+yb;
    dyb = vyb;
}

void LIP::step8() {
    pxa = -a*(C-1)/D*(xd-C*xi-Tc*S*dxi) - b*S/(Tc*D)*(dxb-S/Tc*xi-C*dxi);
    pya = -a*(C-1)/D*(yd-C*yi-Tc*S*dyi) - b*S/(Tc*D)*(dyb-S/Tc*yi-C*dyi);
    pxa_list.push_back(pxa); pya_list.push_back(pya);
}

void LIP::graph_show() {
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::plot(x_list, y_list);
    plt::xlim(-0.1,1.0);
    plt::ylim(-0.05,0.25);
    plt::plot(px_list, py_list, "x");
    plt::plot(pxa_list, pya_list, "x");     //着地点をプロット
    plt::show();
}

int main()
{
    LIP lip(0.8, 0.02, 10, 1);
    lip.step2();

    for(int n=0; n<6; n++) {
        lip.step3_and_4(n);
        lip.step5();
        lip.step6();
        lip.step7();
        lip.step8();
    }

    lip.graph_show();

    return 0;
}
