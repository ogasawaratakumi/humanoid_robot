#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Link.h"
#include "Jacobian.h"

class Kinematics
{
public:
	struct Link *ulink;
public:
	Kinematics(Link *ulink)
	{
		this->ulink = ulink;
	}
	void calcForwardKinematics(int rootlink);
	bool calcInverseKinematics(int to, Link target);

	Matrix<double,3,3> computeMatrixFromAngles(double r, double p, double y);
	void computeAnglesFromMatrix(Matrix<double,3,3> R, double &r, double &p, double &y);
	vector<int> FindRoute(int to);
	Matrix<double,3,3> Rodrigues(Matrix<double,3,1> a, double q);
	Matrix<double,3,1> rot2omega(Matrix<double,3,3> R);
	Matrix<double,6,1> calcVWerr(Link Cref, Link Cnow);
};

#endif
