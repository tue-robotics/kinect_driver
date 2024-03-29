cmake_minimum_required(VERSION 3.0.2)
project(kinect_driver)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
    rgbd
    roscpp
    sensor_msgs
)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
add_service_files(
   FILES
   SetSettings.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages()

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

find_package(OpenCV REQUIRED)
find_library(freenect_LIBRARIES NAMES libfreenect.so PATHS /usr/lib usr/lib/x86_64-linux-gnu)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(kinect_driver
    src/driver.cpp
)
target_link_libraries(kinect_driver ${catkin_LIBRARIES} ${freenect_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(kinect_driver ${PROJECT_NAME}_gencpp)

