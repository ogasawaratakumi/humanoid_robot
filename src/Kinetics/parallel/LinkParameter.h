#ifndef __LINK_PARAMETER_H__
#define __LINK_PARAMETER_H__

#include <iostream>
#include <string>
#include "../Eigen/Dense"
#include <limits>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace Eigen;

const int NON = -1;
static const double eps = 1e-06;
static const double pi = boost::math::constants::pi<double>();

enum {
    FOOT_ROLL_L = 0,
    KNEE_L1,
    KNEE_L2,
    FOOT_PITCH_L,
    LEG_ROLL_L,
    LEG_YAW_L,
    FOOT_ROLL_R,
    KNEE_R1,
    KNEE_R2,
    FOOT_PITCH_R,
    LEG_YAW_R,

    JOINT_NUM
};

enum {
    BASE = 0,
    RY,
    RR1,
    RP1,
    RP2,
    RP3,
    RP4,
    RR2,
    RF,
    LY,
    LR1,
    LP1,
    LP2,
    LP3,
    LP4,
    LR2,
    LF,

    LINK_NUM
};

static const string joint_name[] = {
    "BASE",
    "RY",
    "RR1",
    "RP1",
    "RP2",
    "RP3",
    "RP4",
    "RR2",
    "RF",
    "LY",
    "LR1",
    "LP1",
    "LP2",
    "LP3",
    "LP4",
    "LR2",
    "LF",
};

struct link_connect {
    int sister;
    int child;
    int parent;
};

static const struct link_connect LINK_CONNECT[LINK_NUM] = {
    { NON, RY , NON  },

    { LY,  RR1, BASE },
    { NON, RP1, RY   },
    { NON, RP2, RR1  },
    { NON, RP3, RP1  },
    { NON, RP4, RP2  },
    { NON, RR2, RP3  },
    { NON, RF,  RP4  },
    { NON, NON, RR2  },

    { NON, LR1, BASE },
    { NON, LP1, LY   },
    { NON, LP2, LR1  },
    { NON, LP3, LP1  },
    { NON, LP4, LP2  },
    { NON, LR2, LP3  },
    { NON, LF,  LP4  },
    { NON, NON, LR2  },
};

static const int LinkAxis[LINK_NUM][3] = {
    {0,0,0},

    {0,0,1},
    {1,0,0},
    {0,1,0},
    {0,1,0},
    {0,1,0},
    {0,1,0},
    {1,0,0},
    {0,0,0},

    {0,0,1},
    {1,0,0},
    {0,1,0},
    {0,1,0},
    {0,1,0},
    {0,1,0},
    {1,0,0},
    {0,0,0},
};

static const double LinkPos[LINK_NUM][3] = {
    {0.0f, 0.0f, 0.0f},

    { 0.0f,  -44.0f,     0.0f},
    { 0.0f,    0.0f,   -42.8f},
    {15.0f,    0.0f,     0.0f},
    { 0.0f,    0.0f,  -108.0f},
    { 0.0f,    0.0f,   -41.0f},
    { 0.0f,    0.0f,  -108.0f},
    { 0.0f,    0.0f,     0.0f},
    { 0.0f,    0.0f,   -43.5f},
    
    { 0.0f,   44.0f,     0.0f},
    { 0.0f,    0.0f,   -42.8f},
    {15.0f,    0.0f,     0.0f},
    { 0.0f,    0.0f,  -108.0f},
    { 0.0f,    0.0f,   -41.0f},
    { 0.0f,    0.0f,  -108.0f},
    { 0.0f,    0.0f,     0.0f},
    { 0.0f,    0.0f,   -43.5f},
};

#endif
