#include <perfect_velodyne/normal_estimator_component.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<perfect_velodyne::NormalEstimatorComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}
