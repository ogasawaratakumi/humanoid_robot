#include "Kinematics.h"

//humanoid robot p53
Matrix<double,3,3> Kinematics::computeMatrixFromAngles( double r, double p, double y ) {
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

void Kinematics::computeAnglesFromMatrix( Matrix<double,3,3> R, double &r, double &p, double &y ) {
    double threshold = 0.001;
    if( abs(R(2,1) - 1.0 ) < threshold ) {
        r = M_PI / 2;
        p = 0;
        y = atan2(R(1,0), R(0,0));
    } else if( abs(R(2,1) + 1.0 ) < threshold ) {
        r = -M_PI / 2;
        p = 0;
        y = atan2(R(1,0), R(0,0));
    } else {
        r = asin(R(2,1));
        p = atan2(-R(2,0), R(2,2));
        y = atan2(-R(0,1), R(1,1));
    }
}

//humanoid robot p34
Matrix<double,3,3> Kinematics::Rodrigues( Matrix<double,3,1> a, double q ) {
    return AngleAxisd(q, a).toRotationMatrix();
}

//humanoid robot p35
Matrix<double,3,1> Kinematics::rot2omega( Matrix<double,3,3> R ) {
    double alpha = (R(0,0)+R(1,1)+R(2,2)-1)/2;
    double th;
    Matrix<double,3,1> vector_R( Matrix<double,3,1>::Zero() );
    
    if( fabs(alpha-1) < eps ) {
        return Matrix<double,3,1>::Zero();
    }

    th = acos(alpha);
    vector_R << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);

    return 0.5*th/sin(th)*vector_R;
}

//humanoid robot p64
vector<int> Kinematics::FindRoute( int to ) {
	vector<int> idx;
	int link_num = to;
	while( link_num != 0 )
	{
		idx.push_back(link_num);
		link_num = ulink[link_num].parent;
	}
	reverse( idx.begin(), idx.end() );
	return idx;
}

//humanoid robot p64
Matrix<double,6,1> Kinematics::calcVWerr( Link Cref, Link Cnow ) {
	Matrix<double,3,1> perr = Cref.p - Cnow.p;
	Matrix<double,3,3> Rerr = Cnow.R.transpose() * Cref.R;
	Matrix<double,3,1> werr = Cnow.R * rot2omega( Rerr );
	Matrix<double,6,1> err;
	err << perr,werr;
	return err;
}

//humanoid robot p50
void Kinematics::calcForwardKinematics( int rootlink ) {
	if( rootlink == -1 ) return;
	if( rootlink != 0 ) {
		int parent = ulink[rootlink].parent;
		ulink[rootlink].p = ulink[parent].p + ulink[parent].R * ulink[rootlink].b;
		ulink[rootlink].R = ulink[parent].R * Rodrigues( ulink[rootlink].a, ulink[rootlink].q );
	}
	calcForwardKinematics(ulink[rootlink].sister);
	calcForwardKinematics(ulink[rootlink].child);
}

//humanoid robot p58
bool Kinematics::calcInverseKinematics( int to, Link target ) {
	MatrixXd J, dq;
	Matrix<double,6,1> err;

	ColPivHouseholderQR<MatrixXd> QR; //QR分解?
	const double dampingConstantSqr = 1.0e-12;
	const double lambda = 0.5;
	const int iteration = 100;

	calcForwardKinematics(BASE);

	vector<int> idx = FindRoute(to);
	const int jsize = idx.size();

	J.resize(6,jsize); dq.resize(jsize,1);

	for( int n=0; n<iteration; n++ ) {
		J = calcJacobian(ulink, idx);
		err = calcVWerr(target, ulink[to]);
		if( err.norm() < eps ) return true;

		MatrixXd JJ = J*J.transpose()+dampingConstantSqr*MatrixXd::Identity(J.rows(),J.rows());
		dq = J.transpose() * QR.compute(JJ).solve(err) * lambda;

		for( size_t nn=0; nn<jsize; nn++ ) {
			int j = idx[nn];
			ulink[j].q += dq(nn);
		}
		calcForwardKinematics(BASE);
	}
	return false;
}

void Kinematics::MoveJoints( vector<int> idx, MatrixXd dq ) {
	for( int n=0; n<idx.size(); n++ )
	{
		int j = idx[n];
		ulink[j].q += dq(n);
	}
}


