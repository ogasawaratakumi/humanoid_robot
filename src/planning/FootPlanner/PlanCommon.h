#ifndef _PLANCOMMON_H__
#define _PLANCOMMOM_H__

enum FootStatus
{
  BothLeg = 0,
  RightLeg = 1,
  LeftLeg = -1,
};

enum WalkingStatus
{
  Start = 0,
  Walking = 1,
  Stop = 2,
};

typedef struct
{
  double x;
  double y;
  double th;
}Pos2D;

#endif
