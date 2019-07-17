

#ifndef SIMP_WM_HH
#define SIMP_WM_HH

#include "corridor.h"
#include "elevator.h"
#include "datatypes_wm.h"

#include <geometry_msgs/PoseStamped.h>

namespace wm
{
class SimplifiedWorldModel
{


public:
    SimplifiedWorldModel();
    ~SimplifiedWorldModel();

    Corridor corridor1;
    Elevator elevator1;



};

PoseWm getWMPose(const geometry_msgs::PoseStamped &pose);

}
#endif
