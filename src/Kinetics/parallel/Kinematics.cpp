#include "Kinematics.h"

Matrix<double,3,3> Kinematics::computeMatrixFromAngles(double r, double p, double y)
{
	Matrix<double,3,3> R;

	R(0,0) = cos(p) * cos(y) - sin(r) * sin(p) * sin(y);
	R(0,1) = -cos(r) * sin(y);
	R(0,2) = sin(r) * cos(y) + sin(r) * cos(p) * sin(y);
	R(1,0) = cos(p) * sin(y) + sin(r) * sin(p) * cos(y);
	R(1,1) = cos(r) * cos(y);
	R(1,2) = sin(p) * sin(y) - sin(r) * cos(p) * cos(y);
	R(2,0) = -cos(r) * sin(p);
	R(2,1) = sin(r);
	R(2,2) = cos(r) * cos(p);	

	return R;
}

void Kinematics::computeAnglesFromMatrix(Matrix<double,3,3> R, double &r, double &p, double &y)
{
	double threshold = 0.001;
	if(abs(R(2,1) - 1.0) < threshold){ // R(2,1) = sin(x) = 1の時
		r = pi / 2;
		p = 0;
		y = atan2(R(1,0), R(0,0));
	}else if(abs(R(2,1) + 1.0) < threshold){ // R(2,1) = sin(x) = -1の時
		r = - pi / 2;
		p = 0;
		y = atan2(R(1,0), R(0,0));
	}else{
		r = asin(R(2,1));
		p = atan2(-R(2,0), R(2,2));
		y = atan2(-R(0,1), R(1,1));
	}
}

Matrix<double,3,3>Kinematics::Rodrigues( Matrix<double,3,1> a, double q )
{
	return AngleAxisd(q,a).toRotationMatrix();
}

Matrix<double,3,1> Kinematics::rot2omega(Matrix<double,3,3> R)
{
	double alpha = (R(0,0)+R(1,1)+R(2,2)-1)/2;
	double th;
	Matrix<double,3,1> vector_R(Matrix<double,3,1>::Zero());

	if(fabs(alpha-1) < eps)
		return Matrix<double,3,1>::Zero();

	th = acos(alpha);
	vector_R << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);
	return 0.5*th/sin(th)*vector_R;
}

vector<int> Kinematics::FindRoute(int to)
{
	vector<int> idx;
	int link_num = to;
	while( link_num != 0)
	{
		if((link_num != RP2) && (link_num != LP2))
			idx.push_back(link_num);
		link_num = ulink[link_num].parent;
	}
	reverse(idx.begin(), idx.end());
	return idx;
}

Matrix<double,6,1> Kinematics::calcVWerr(Link Cref, Link Cnow)
{
	Matrix<double,3,1> perr = Cref.p - Cnow.p;
	Matrix<double,3,3> Rerr = Cnow.R.transpose() * Cref.R;
	Matrix<double,3,1> werr = Cnow.R * rot2omega(Rerr);
	Matrix<double,6,1> err;
	err << perr,werr;
	return err;
}

void Kinematics::calcForwardKinematics( int rootlink)
{
	if(rootlink == -1) return;
	if(rootlink != 0)
	{
		int parent = ulink[rootlink].parent;
		ulink[rootlink].p = ulink[parent].p + ulink[parent].R * ulink[rootlink].b;
		ulink[rootlink].R = ulink[parent].R * Rodrigues( ulink[rootlink].a, ulink[rootlink].q );

	}
	calcForwardKinematics(ulink[rootlink].sister);
	calcForwardKinematics(ulink[rootlink].child);
}

bool Kinematics::calcInverseKinematics(int to, Link target)
{
	MatrixXd J, dq;
	Matrix<double,6,1> err;

	ColPivHouseholderQR<MatrixXd> QR;
	const double dampingConstantSqr = 1.0e-12;
	const double lambda = 0.5;
	const int iteration = 100;

	vector<int> idx = FindRoute(to);
	const int jsize = idx.size();
	J.resize(6,jsize); dq.resize(jsize,1);

	for(int n=0;n<iteration;n++){
		calcForwardKinematics(BASE);
		err = calcVWerr(target, ulink[to]);
		if(err.norm() < eps) return true;
		J = calcJacobian(ulink, idx);
		MatrixXd JJ = J*J.transpose()+dampingConstantSqr*MatrixXd::Identity(J.rows(),J.rows());
		dq = J.transpose() * QR.compute(JJ).solve(err) * lambda;

		for(size_t nn=0;nn<jsize;nn++){
			int j = idx[nn];
			ulink[j].q += dq(nn);
		}
		if(to == RR2) ulink[RP2].q = -ulink[RP1].q;
		if(to == LR2) ulink[LP2].q = -ulink[LP1].q;
	}
	return false;
}


/*
bool Kinematics::calcInverseKinematics( int to, Link target ) {
  Matrix<double,6,6> J;
  Matrix<double,6,1> dq, err;

  const double lambda = 0.5;
  const int iteration = 100;

  calcForwardKinematics(BASE);
  vector<int> idx = FindRoute(to);

  for(int n=0;n<iteration;n++){
	J = calcJacobian2(ulink, idx);
	err = calcVWerr(target, ulink[to]);
	if(err.norm() < eps) return true;
	dq = lambda * (J.inverse() * err);
	for(size_t nn=0;nn<idx.size();nn++){
	  int j = idx[nn];
	  ulink[j].q += dq(nn);
	}
	if(to == RR2) ulink[RP2].q = -ulink[RP1].q;
	else if(to == LR2) ulink[LP2].q = -ulink[LP1].q;
	calcForwardKinematics(BASE);
  }
  return false;
}
*/
