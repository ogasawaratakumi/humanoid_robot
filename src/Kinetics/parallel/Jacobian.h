#ifndef __JACABIAN_H__
#define __JACABIAN_H__

#include <iostream>
#include "../Eigen/Dense"
#include "Link.h"

using namespace Eigen;

Matrix<double,3,1> calcMC( Link *ulink, int rootlink );
double calcTotalMass( Link *ulink, int rootlink );
Matrix<double,3,1> calcCoM( Link *ulink );
MatrixXd calcJacobian( Link *ulink, std::vector<int> idx );
MatrixXd calcJacobian2( Link *ulink, std::vector<int> idx );

#endif
