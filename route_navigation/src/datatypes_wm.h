

#ifndef DATATYPES_WM_HH
#define DATATYPES_WM_HH

#include <vector>

namespace wm
{
  struct point_wm
  {
	double x;
	double y;
  };
  
  struct pose_wm
  {
      struct{
	double x;
	double y;
	double z;
      } position;
      struct{
	double x;
	double y;
	double z;
	double w;
      } orientation;      
  };  
  
  typedef struct point_wm point_wm;
  typedef struct pose_wm pose_wm;
    
}
#endif
