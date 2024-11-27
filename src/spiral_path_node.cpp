#include "Spiral_Path.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spiral_planner");
    CShort_Distance_Planner iSDP;

    ros::spin();

    return 0;
}
