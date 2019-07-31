#include <iostream>
#include <fstream>
#include <cmath>
#include <GL/glut.h>

#include "Kinematics.h"
#include "Link.h"
#include "../Eigen/Dense"

double deg2rad( double deg ) {
  return M_PI*deg/180.0f;
}

double rad2deg( double rad ) {
  return 180.0f*rad/M_PI;
}

int main() {
  std::vector<double> x_list, z_list, x1_list, z1_list, x2_list, z2_list, x3_list, z3_list, x4_list, z4_list;
  std::vector<double> place_x, place_z, place_x1, place_z1, place_x2, place_z2;
  const char *fileName = "data.txt";
  std::ofstream ofs(fileName);

  int i, j;
  const int row = 4;
  const int colm = 3;

  double base[row][colm] = {
	{ 0.0,  0.0,  0.0 },
	{ 0.0,  0.44, 0.0 },
	{ 0.0, -0.44, 0.0 }
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
  for(int i=BASE; i<=RF2; i++) {
	std::cout << i << " = " << ulink[i].p.transpose() << std::endl;
  }
  return 0;
}
