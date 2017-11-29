

#ifndef SIMP_WM_HH
#define SIMP_WM_HH

#include "corridor.h"
#include "elevator.h"
#include "datatypes_wm.h"

#include <geometry_msgs/PoseStamped.h>

namespace wm
{
  class Simplified_WorldModel
  {
    
    
    public: 
      Simplified_WorldModel();
      ~Simplified_WorldModel();
      
      Corridor corridor1;
      Elevator elevator1;
    
      

  };
  
  pose_wm getWMPose(const geometry_msgs::PoseStamped &pose);
  
}
#endif
