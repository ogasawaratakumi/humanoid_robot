#include "Kinematics.h"
#include "Link.h"
#include "../Eigen/Dense"

#include <iostream>
#include <fstream>
#include <cmath>

double deg2rad( double deg ) {
    return M_PI*deg/180.0f;
}

double rad2deg( double rad ) {
    return 180.0f*rad/M_PI;
}

int main() {
    const char *fileName = "data.txt";
    std::ofstream ofs( fileName );
    Link ulink[JOINT_NUM];
    Kinematics kine( ulink );
    SetJointInfo( ulink );

    for( int i=0; i<JOINT_NUM; i++ ) {
        ulink[i].q = 0.0;
    }

    ulink[RLEG_J0].q = 0.0;
    ulink[RLEG_J1].q = 0.0;
    ulink[RLEG_J2].q = deg2rad(-30);
    ulink[RLEG_J3].q = deg2rad(60);
    ulink[RLEG_J4].q = deg2rad(-30);
    ulink[RLEG_J5].q = 0.0;

    ulink[LLEG_J0].q = 0.0;
    ulink[LLEG_J1].q = 0.0;
    ulink[LLEG_J2].q = deg2rad(-30);
    ulink[LLEG_J3].q = deg2rad(60);
    ulink[LLEG_J4].q = deg2rad(-30);
    ulink[LLEG_J5].q = 0.0;

    kine.calcForwardKinematics(BASE);

    /*
       std::cout << "RLEG_J0 q = " << ulink[RLEG_J0].p.transpose() << std::endl;
       std::cout << "RLEG_J1 q = " << ulink[RLEG_J1].p.transpose() << std::endl;
       std::cout << "RLEG_J2 q = " << ulink[RLEG_J2].p.transpose() << std::endl;
       std::cout << "RLEG_J3 q = " << ulink[RLEG_J3].p.transpose() << std::endl;
       std::cout << "RLEG_J4 q = " << ulink[RLEG_J4].p.transpose() << std::endl;
       std::cout << "RLEG_J5 q = " << ulink[RLEG_J5].p.transpose() << std::endl;
       */
    //kine.calcInverseKinematics( 0, BASE );

    ofs << ulink[BASE].p.transpose() << std::endl;
    ofs << ulink[RLEG_J0].p.transpose() << std::endl;
    ofs << ulink[LLEG_J0].p.transpose() << std::endl;
    ofs << std::endl;

    for( int r_leg=1; r_leg<6; r_leg++ ) {
        ofs << ulink[r_leg].p.transpose() << std::endl;
    }
    ofs << std::endl;

    for( int l_leg=7; l_leg<12; l_leg++ ) {
        ofs << ulink[l_leg].p.transpose() << std::endl;
    }

    return 0;
}
