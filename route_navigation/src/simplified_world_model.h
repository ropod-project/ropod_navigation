

#ifndef SIMP_WM_HH
#define SIMP_WM_HH

#include "corridor.h"
#include "elevator.h"
#include "datatypes_wm.h"

namespace wm
{
  class Simplified_WorldModel
  {
    
    
    public: Simplified_WorldModel();

    public: ~Simplified_WorldModel();
    
    public: Corridor corridor1;
    public: Elevator elevator1;


  };
  
}
#endif
