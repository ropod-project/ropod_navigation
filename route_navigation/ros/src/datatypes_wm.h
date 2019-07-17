

#ifndef DATATYPES_WM_HH
#define DATATYPES_WM_HH

#include <vector>

namespace wm
{
  struct PointWm
  {
	double x;
	double y;
  };
  
  struct PoseWm
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
  
  typedef struct PointWm PointWm;
  typedef struct PoseWm PoseWm;
    
}
#endif
