#include <perfect_velodyne.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perfect_velodyen");

    PERFECT_VELODYNE::NormalEstimation NormalEstimation;

    ros::Rate rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
