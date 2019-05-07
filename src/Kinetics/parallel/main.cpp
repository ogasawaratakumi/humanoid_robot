#include <iostream>
#include <fstream>
#include <cmath>

#include "Kinematics.h"
#include "Link.h"
#include "../Eigen/Dense"
//#include "matplotlibcpp.h"

//namespace plt = matplotlibcpp;

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

    //TODO LinkParameter.hに移行
    
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
    ulink[LP4].q = -deg2rad(20.0) + deg2rad(0.0);
    ulink[LR2].q = deg2rad(0.0);

    ulink[RY ].q = deg2rad(0.0);
    ulink[RR1].q = deg2rad(0.0);
    ulink[RP1].q = deg2rad(-20.0);
    ulink[RP2].q = -deg2rad(-20.0);
    ulink[RP3].q = deg2rad(20.0);
    ulink[RP4].q = -deg2rad(20.0) + deg2rad(0.0);
    ulink[RR2].q = deg2rad(0.0);

    kine.calcForwardKinematics(BASE);

    /*
    std::cout << ulink[BASE].p.transpose() << std::endl;
    std::cout << ulink[RY  ].p.transpose() << std::endl;
    std::cout << ulink[RR1 ].p.transpose() << std::endl;
    std::cout << ulink[RP1 ].p.transpose() << std::endl;
    std::cout << ulink[RP2].p.transpose() << std::endl;
    std::cout << ulink[RP3].p.transpose() << std::endl;
    std::cout << ulink[RP4].p.transpose() << std::endl;
    std::cout << ulink[RR2].p.transpose() << std::endl;
    std::cout << ulink[RF ].p.transpose() << std::endl;
    */

    for( i=0; i<row; i++ ) {
        for( j=0; j<colm; j++ ) {
            ofs << base[i][j] << ' ';
        }
        if( j == row-1 ) {
            ofs << std::endl;
        }
    }

    /*
    for( int r_leg=1; r_leg<9; r_leg++ ) {
        ofs << ulink[r_leg].p.transpose() << std::endl;
    }
    ofs << std::endl;

    for( int l_leg=9; l_leg<17; l_leg++ ) {
        ofs << ulink[l_leg].p.transpose() << std::endl;
    }
    */
    
    int target_link=8;
    Link Target = ulink[target_link];

    Target.p << 0.084, -0.44, -3.39;

    kine.calcInverseKinematics(target_link, Target);

    std::cout << ulink[BASE].p.transpose() << std::endl;
    std::cout << ulink[RR1].p.transpose() << std::endl;
    std::cout << ulink[RP1].p.transpose() << std::endl;
    std::cout << ulink[RP2].p.transpose() << std::endl;
    std::cout << ulink[RP3].p.transpose() << std::endl;
    std::cout << ulink[RP4].p.transpose() << std::endl;
    std::cout << ulink[RR2].p.transpose() << std::endl;
    std::cout << ulink[RF].p.transpose() << std::endl;
    
    z_list.push_back((ulink[BASE].p)(2));
    z_list.push_back((ulink[RY  ].p)(2));
    z_list.push_back((ulink[RR1 ].p)(2));
    z_list.push_back((ulink[RP1 ].p)(2));
    z_list.push_back((ulink[RP2 ].p)(2));
    z_list.push_back((ulink[RP3 ].p)(2));
    z_list.push_back((ulink[RP4 ].p)(2));
    z_list.push_back((ulink[RR2 ].p)(2));
    z_list.push_back((ulink[RF  ].p)(2));

    x_list.push_back((ulink[BASE].p)(0));
    x_list.push_back((ulink[RY  ].p)(0));
    x_list.push_back((ulink[RR1 ].p)(0));
    x_list.push_back((ulink[RP1 ].p)(0));
    x_list.push_back((ulink[RP2 ].p)(0));
    x_list.push_back((ulink[RP3 ].p)(0));
    x_list.push_back((ulink[RP4 ].p)(0));
    x_list.push_back((ulink[RR2 ].p)(0));
    x_list.push_back((ulink[RF  ].p)(0));
    
     for( int r_leg=1; r_leg<9; r_leg++ ) {
        ofs << ulink[r_leg].p.transpose() << std::endl;
    }
    ofs << std::endl;

    for( int l_leg=9; l_leg<17; l_leg++ ) {
        ofs << ulink[l_leg].p.transpose() << std::endl;
    }

    /*
    plt::plot(x_list, z_list);
    plt::show();
    */

    return 0;
}
