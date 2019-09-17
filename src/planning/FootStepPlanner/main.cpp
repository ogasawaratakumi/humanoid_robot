#include "FootStepPlanner.h"

const int MAX_STEP = 15;

int main( int argc, char *argv[] ) {
  int count = 0;
  Vector2d tP_B( Vector2d::Zero() );
  double tth_B = 0.0;

  FootStepPlanner plan_node( 0.32, 0.05 );

  while(1) {
	plan_node.getNextStep( tP_B, tth_B );
	if( plan_node.getWalkState() == STOP ) break;

	count++;
	if( MAX_STEP <= count ) plan_node.setStopFlag();

	tP_B += Vector2d( 0.03, 0.03 ); tth_B += 5.0;
	//std::cout << "tP_B = " << tP_B << std::endl;
	//std::cout << "tth_B = " << tth_B << std::endl;
  }
  plan_node.plot_foot_pattern();
}
