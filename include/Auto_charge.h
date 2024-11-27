#ifndef _AUTO_CHARGE_H
#define _AUTO_CHARGE_H

#include "Spiral_Path.h"

class CAuto_Charge_Planner : public CShort_Distance_Planner
{
  public:
  private:
    bool spiral_plan_srv_callback(spiral_planner::spiral::Request& request,
                                  spiral_planner::spiral::Response& response) override;
};

#endif