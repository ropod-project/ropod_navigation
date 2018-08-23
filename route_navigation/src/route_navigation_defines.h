

#ifndef ROUTE_NAV_DEF_HH
#define ROUTE_NAV_DEF_HH

#include <geometry_msgs/Wrench.h>



enum
{
    NAV_HOLD = 0,
    NAV_IDLE,
    NAV_PAUSED,
    NAV_BUSY,
    NAV_GETPOINT,
    NAV_GOTOPOINT,
    NAV_WAYPOINT_DONE,
    NAV_DONE
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
    LLC_NORMAL = 0,             // vel_control
    LLC_FX_FY_DT,               // Force-control in x and y, theta disabled
    LLC_VX_FY_DT,               // vel control in x, force control in y, theta disabled
    LLC_DOCKING    
};

struct wrenches{
   geometry_msgs::Wrench front;     
   geometry_msgs::Wrench left;
   geometry_msgs::Wrench back;
   geometry_msgs::Wrench right;
};
}

#endif
