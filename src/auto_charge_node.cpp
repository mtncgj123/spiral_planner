#include "Auto_charge.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_charge");
    CAuto_Charge_Planner iACP;
    MINF("auto_charge node init");

    ros::spin();

    return 0;
}