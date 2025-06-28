cmake_minimum_required(VERSION 3.10)

# Basic example
add_executable(basic_example basic_example.cpp)
target_link_libraries(basic_example ${PROJECT_NAME})

# Velocity control example
add_executable(velocity_control_example velocity_control_example.cpp)
target_link_libraries(velocity_control_example ${PROJECT_NAME})

# Position control example
add_executable(position_control_example position_control_example.cpp)
target_link_libraries(position_control_example ${PROJECT_NAME})

# Multi-motor example
add_executable(multi_motor_example multi_motor_example.cpp)
target_link_libraries(multi_motor_example ${PROJECT_NAME})
