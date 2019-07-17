#include "corridor.h"


namespace wm
{
    Corridor::Corridor(){
      
    };
    void Corridor:: init( double* line1_, double* line2_, double* wayp_line1_, double* wayp_line2_){
      this->line1.assign(line1_ , line1_ + 3) ;
      this->line2.assign(line2_ , line2_ + 3) ;
      this->wayp_line1.assign(wayp_line1_ , wayp_line1_ + 3) ;
      this->wayp_line2.assign(wayp_line2_ , wayp_line2_ + 3) ;
      
    };
    
    Corridor::~Corridor(){
      };

}
  
  
