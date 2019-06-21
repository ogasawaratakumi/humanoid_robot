#include <iostream>
#include <fstream>
#include <cmath>
#include <GL/glut.h>

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
  std::vector<double> x_list, z_list, x1_list, z1_list, x2_list, z2_list, x3_list, z3_list, x4_list, z4_list;
  std::vector<double> place_x, place_z, place_x1, place_z1, place_x2, place_z2;
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
  ulink[LP1].q = deg2rad(-50.0);
  ulink[LP2].q = -deg2rad(-50.0);
  ulink[LP3].q = deg2rad(50.0);
  ulink[LP4].q = -deg2rad(50.0);
  ulink[LR2].q = deg2rad(0.0);

  ulink[RY ].q = deg2rad(0.0);
  ulink[RR1].q = deg2rad(0.0);
  ulink[RP1].q = deg2rad(-50.0);
  ulink[RP2].q = -deg2rad(-50.0);
  ulink[RP3].q = deg2rad(50.0);
  ulink[RP4].q = -deg2rad(50.0);
  ulink[RR2].q = deg2rad(0.0);

  kine.calcForwardKinematics(BASE);

  for( int i=RY; i<=RF2; i++ ) {
	x_list.push_back(ulink[i].p(0));
	z_list.push_back(ulink[i].p(2));
  }

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

	Target.p << -0.607, -0.44, -2.34;
	Target.R = kine.computeMatrixFromAngles( deg2rad(0), deg2rad(0), deg2rad(-5) );

	kine.calcInverseKinematics(target_link, Target);
	for( int i=RY; i<=RF2; i++ ) {
	  x1_list.push_back(ulink[i].p(0));
	  z1_list.push_back(ulink[i].p(2));
	  if(i==RR2){
		cout << ulink[RR2].p(0) << endl;
		cout << ulink[RR2].p(2) << endl;
	  }
	}

  Link Target_r2 = ulink[target_link];

  Target.p << 1.28, -0.44, -2.765;
  Target.R = kine.computeMatrixFromAngles( deg2rad(0), deg2rad(20), deg2rad(0) );

  kine.calcInverseKinematics(target_link, Target);
  for( int i=RY; i<=RF2; i++ ) {
	x2_list.push_back(ulink[i].p(0));
	z2_list.push_back(ulink[i].p(2));
	if(i==RR2) {
	  cout << ulink[RR2].p(0) << endl;
	  cout << ulink[RR2].p(2) << endl;
	}
  }
  
  Link Target_r3 = ulink[target_link];
  Target.p << 1.78, -0.44, -2.065;
  Target.R = kine.computeMatrixFromAngles( deg2rad(0), deg2rad(0), deg2rad(0) );

  kine.calcInverseKinematics(target_link, Target);
  for( int i=RY; i<=RF2; i++ ) {
	x3_list.push_back(ulink[i].p(0));
	z3_list.push_back(ulink[i].p(2));
  	if(i==RR2) {
	  cout << ulink[RR2].p(0) << endl;
	  cout << ulink[RR2].p(2) << endl;
	}
  }

  Link Target_r4 = ulink[target_link];
  Target.p << -0.83, -0.44, -2.025;
  Target.R = kine.computeMatrixFromAngles( deg2rad(0), deg2rad(45), deg2rad(0) );

  kine.calcInverseKinematics(target_link, Target);
  for( int i=RY; i<=RF2; i++ ) {
	x4_list.push_back(ulink[i].p(0));
	z4_list.push_back(ulink[i].p(2));
	if(i==RR2) {
	  cout << ulink[RR2].p(0) << endl;
	  cout << ulink[RR2].p(2) << endl;
	}
  }

  target_link = LR2;
  Link Target_l = ulink[target_link];

  Target_l.p << 0.22, 0.44, -1.60;
  Target_l.R = kine.computeMatrixFromAngles( deg2rad(0), deg2rad(0), deg2rad(0) );

  kine.calcInverseKinematics(target_link, Target_l);

  for( int r_leg=RY; r_leg<=RF; r_leg++ ) {
	ofs << ulink[r_leg].p.transpose() << std::endl;
  }
  ofs << std::endl;

  for( int l_leg=LY; l_leg<=LF; l_leg++ ) {
	ofs << ulink[l_leg].p.transpose() << std::endl;
  }
  place_x.push_back(-0.6069); place_z.push_back(-2.34);
  place_x1.push_back(-0.6069); place_z1.push_back(-2.34);
  place_x2.push_back(-0.6069); place_z2.push_back(-2.34);
  place_x.push_back(1.20536); place_z.push_back(-2.60992);
  place_x1.push_back(1.20536); place_z1.push_back(-2.60992);
  place_x2.push_back(1.20536); place_z2.push_back(-2.60992);
  place_x.push_back(1.72385); place_z.push_back(-2.09873);
  place_x1.push_back(1.72385); place_z1.push_back(-2.09873);
  place_x2.push_back(1.72385); place_z2.push_back(-2.09873);
  place_x.push_back(-0.83); place_z.push_back(-2.025);
  place_x1.push_back(-0.83); place_z1.push_back(-2.025);
  place_x2.push_back(-0.83); place_z2.push_back(-2.025);
  
  plt::subplot(1,2,1);
  plt::named_plot( "leg model", x_list, z_list);
  plt::xlim(-1.5, 2.5);
  plt::ylim(-3.5, 3.5 );
  plt::legend(); 
  plt::subplot(1,2,2);
  plt::named_plot( "Frame1", x4_list, z4_list, "--m");
  plt::named_plot( "Frame2", x1_list, z1_list, "--b");
  plt::named_plot( "Frame3", x2_list, z2_list, "--r");
  plt::named_plot( "Frame4", x3_list, z3_list, "--g");
  plt::plot(  place_x, place_z, "x");
  plt::plot(  place_x1, place_z1, "x");
  plt::named_plot( "Target", place_x2, place_z2, "x");
  plt::legend(); 
  plt::xlim(-1.5, 2.5);
  plt::ylim(-3.5, 3.5 );
  plt::show();

  return 0;
}
