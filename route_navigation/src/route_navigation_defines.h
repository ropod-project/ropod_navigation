

#ifndef ROUTE_NAV_DEF_HH
#define ROUTE_NAV_DEF_HH

#include <geometry_msgs/WrenchStamped.h>



enum
{
    NAV_HOLD = 0,
    NAV_IDLE,
    NAV_PAUSED,
    NAV_BUSY,
    NAV_GETPOINT,
    NAV_GOTOPOINT,
    NAV_WAYPOINT_DONE,
    NAV_DOCKED,
    NAV_UNDOCKED,
    NAV_DONE
};

/*
 * Additional status codes for feedback from Status.msg
 */
 enum {
	// Generic codes
	UNDEFINED=0,
	FAILED=1,
	SUCCEEDED=2,
	INITIALIZED=3,
	CONFIGURING=4,
	READY=5,
	RUNNING=6,
	RECOVERING=7,
	STOPPED=8,
	// Docking specific codes
	DOCKING_SEQUENCE_SUCCEEDED=9, //a.k.a. DOCKED
	DOCKING_SEQUENCE_FAILED=10,
	COUPLING_SUCCEEDED=11,
	COUPLING_FAILED=12,
	DECOUPLING_SUCCEEDED=13,
	DECOUPLING_FAILED=14,
	UNDOCKING_SEQUENCE_SUCCEEDED=15,
	UNDOCKING_SEQUENCE_FAILED=16,
	MOBIDIK_DETECTED=17,
	MULTIBLE_MOBIDIK_CANDIDATES_DETECTED=18,
	NO_MOBIDIK_DETECTED=19,
	DOCKING_AREA_EMPTY=20, // no MOBIDIK at all
	PRE_DOCK_POSITION_UNREACHABLE=21,
	POST_DOCK_POSITION_UNREACHABLE=22,
	MOBIDIK_UNREACHABLE=23,
	ALIGNMENT_TO_MOBIDIK_FAILED=24,
	DOCKED_TO_WRONG_SIDE=25,
	MOBIDIK_CAN_NOT_BE_PULLED=26, // e.g. we can't pull it out
	ACTIVE_SEARCH_MODE=27,
 };

struct TaskFeedbackCcu
{
    int wayp_n; // waypoint number
    int fb_nav; // status navigation
};

typedef struct TaskFeedbackCcu TaskFeedbackCcu;

namespace ropodNavigation{

  enum LLC
    {
    LLC_VEL = 0,                // vel-control without bumper
    LLC_NORMAL ,                // vel_control with bumper
    LLC_FX_FY_DT,               // Force-control in x and y, theta disabled
    LLC_VX_FY_DT,               // vel control in x, force control in y, theta disabled
    LLC_DOCKING    
};

struct wrenches{
   geometry_msgs::WrenchStamped front;     
   geometry_msgs::WrenchStamped left;
   geometry_msgs::WrenchStamped back;
   geometry_msgs::WrenchStamped right;
};
}

#endif
