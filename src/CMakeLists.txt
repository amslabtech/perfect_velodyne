add_library(normal_estimator_component SHARED
    normal_estimator_component.cpp
)

target_compile_definitions(normal_estimator_component PRIVATE "PERFECT_VELODYNE_NORMAL_ESTIMATOR_BUILDING_DLL")

target_link_libraries(normal_estimator_component
    ${PCL_LIBRARIES}
)

ament_target_dependencies(normal_estimator_component
    rclcpp
    rclcpp_components
    sensor_msgs
    visualization_msgs
)

add_executable(normal_estimator_node
    normal_estimator_node.cpp
)

target_link_libraries(normal_estimator_node
    normal_estimator_component
    ${PCL_LIBRARIES}
)

ament_target_dependencies(normal_estimator_node
    rclcpp
    rclcpp_components
    sensor_msgs
    visualization_msgs
)

install(TARGETS
    normal_estimator_node
    DESTINATION lib/perfect_velodyne
)

install(TARGETS
    normal_estimator_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_libraries(normal_estimator_component)
