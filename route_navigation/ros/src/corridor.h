

#ifndef CORR_WM_HH
#define CORR_WM_HH

#include <vector>

namespace wm
{
  class Corridor
  {
    
    public: Corridor();		      

    public: ~Corridor();

    public: void init( double* line1_, double* line2_, double* wayp_line1_, double* wayp_line2_);
    private: std::vector<double> line1;
    private: std::vector<double> wayp_line1;
    private: std::vector<double> line2;
    private: std::vector<double> wayp_line2;


  };
  
}
#endif
