cmake_minimum_required(VERSION 2.8.3)
project(refollow_path_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            nav_core
            pluginlib
            roscpp
            tf
            nav_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES refollow_path_recovery
    CATKIN_DEPENDS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf
)

add_library(refollow_path_recovery src/refollow_path_recovery.cpp)
add_dependencies(refollow_path_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(refollow_path_recovery ${catkin_LIBRARIES})


install(TARGETS refollow_path_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES ccr_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
