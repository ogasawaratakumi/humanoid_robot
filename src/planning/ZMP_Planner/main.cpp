#include "ZMPPlanner.h"

const int MAX_STEP = 6;

int main( int argc, char *argv[] )
{
  int count = 0;
  Vector2d tP_B( Vector2d(0.0, 0.0) );
  double tth_B = 0.0;

  ZMPPlanner plan_node( 0.32, 0.04 );

  plan_node.refzmp_list.push_back( Vector3d(0.0, 0.0, 0.0) );

  tP_B += Vector2d(0.0, 0.0);

  while(1) {
	plan_node.getNextZMP( tP_B, tth_B );
	if( plan_node.getWalkState() == STOP ) break;

	count++;
	if( MAX_STEP <= count ) plan_node.setStopFlag();

	tP_B += Vector2d(0.05, 0.0); tth_B += 0.0;
  }

  plan_node.plot_foot_pattern();

  return 0;
}
