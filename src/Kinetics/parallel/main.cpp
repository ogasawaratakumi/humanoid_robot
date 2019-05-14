#include <iostream>
#include <fstream>
#include <cmath>

#include "Kinematics.h"
#include "Link.h"
#include "../Eigen/Dense"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

double deg2rad( double deg ) {
    return M_PI*deg/180.0f;
}

double rad2deg( double rad ) {
    return 180.0f*rad/M_PI;
}

int main() {
    std::vector<double> x_list, z_list;
    const char *fileName = "data.txt";
    std::ofstream ofs(fileName);

    int i, j;
    const int row = 4;
    const int colm = 3;

    double base[row][colm] = {
        {0.0,  0.0,  0.0},
        {0.0,  0.44, 0.0},
        {0.0, -0.44, 0.0}
    };
    
    Link ulink[LINK_NUM];
    Kinematics kine(ulink);
    SetJointInfo(ulink);

    ulink[LY ].q = deg2rad(0.0);
    ulink[LR1].q = deg2rad(0.0);
    ulink[LP1].q = deg2rad(-20.0);
    ulink[LP2].q = -deg2rad(-20.0);
    ulink[LP3].q = deg2rad(20.0);
    ulink[LP4].q = -deg2rad(20.0);
    ulink[LR2].q = deg2rad(0.0);

    ulink[RY ].q = deg2rad(0.0);
    ulink[RR1].q = deg2rad(0.0);
    ulink[RP1].q = deg2rad(-20.0);
    ulink[RP2].q = -deg2rad(-20.0);
    ulink[RP3].q = deg2rad(20.0);
    ulink[RP4].q = -deg2rad(20.0);
    ulink[RR2].q = deg2rad(0.0);

    kine.calcForwardKinematics(BASE);

    for( i=0; i<row; i++ ) {
        for( j=0; j<colm; j++ ) {
            ofs << base[i][j] << ' ';
        }
        if( j == row-1 ) {
            ofs << std::endl;
        }
    }
   
    int target_link = RR2;
    Link Target = ulink[target_link];

    Target.p << 0.14, -0.42, -2.60;
    Target.R = kine.computeMatrixFromAngles( deg2rad(0), deg2rad(0), deg2rad(-20) );

    kine.calcInverseKinematics(target_link, Target);

    target_link = LR2;
    Link Target_l = ulink[target_link];

    Target_l.p << 0.12, 0.46, -2.60;
    Target_l.R = kine.computeMatrixFromAngles( deg2rad(0), deg2rad(0), deg2rad(-20) );

    kine.calcInverseKinematics(target_link, Target_l);
    
    for( int i=BASE; i<=LF; i++ ) {
        std::cout << ulink[i].p.transpose() << std::endl;
    }

    for( int i=BASE; i<=RF; i++ ) {
        z_list.push_back((ulink[i].p)(2));
        x_list.push_back((ulink[i].p)(0));
    }

     for( int r_leg=RY; r_leg<RF; r_leg++ ) {
        ofs << ulink[r_leg].p.transpose() << std::endl;
    }
    ofs << std::endl;

    for( int l_leg=LY; l_leg<LF; l_leg++ ) {
        ofs << ulink[l_leg].p.transpose() << std::endl;
    }
     
    plt::plot(x_list, z_list);
    plt::xlim(-0.05,1.0);
    plt::ylim(-3.5, 0.0);
    plt::show();
    
    return 0;
}
