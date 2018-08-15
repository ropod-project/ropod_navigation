

#ifndef ROUTE_NAV_DEF_HH
#define ROUTE_NAV_DEF_HH





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

#endif
