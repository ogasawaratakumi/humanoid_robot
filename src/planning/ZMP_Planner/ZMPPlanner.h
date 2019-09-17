#ifndef __ZMPPLANNER_H__
#define __ZMPPLANNER_H__

#include <iostream>
#include <vector>
#include <cmath>
#include "../Eigen/Dense"

using namespace Eigen;

enum leg_type { RLEG=-1, LLEG=1, BOTH=0 };
enum walk_state { START, WALK, STOP };

class ZMPPlanner
{
  private:
	double foot_time, foot_period;
	double dist_offset, preview_delay;
	Vector2d dP_K;
	Vector2d PFB;
	double dth_K, th_FB;

	leg_type support_leg;
	walk_state status;

  public:
	ZMPPlanner( double _foot_period, double _dist_offset );
	void getNextZMP( Vector2d tP_B, double tth_B );
	void shiftFootVec();
	void setStopFlag() { status = STOP; }
	Vector2d CoordinatesTransform( Vector2d vec, double th );
	leg_type getSupportLeg() { return this->support_leg; }
	walk_state getWalkState() { return this->status; }
	void buffer_clear();
	void plot_foot_pattern();

	std::vector<Vector3d> refzmp_list;
	std::vector<Vector2d> tpb_list;
	std::vector<Vector2d> dpk_list;
};

#endif
