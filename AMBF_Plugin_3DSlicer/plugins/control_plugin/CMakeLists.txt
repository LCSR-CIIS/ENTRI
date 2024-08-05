set(HDR_FILES AMBF_3DSlicer_Plugin.h)
set(SRC_FILES AMBF_3DSlicer_Plugin.cpp)

add_subdirectory(plugin_msgs)
find_package(catkin COMPONENTS plugin_msgs ambf_msgs tf)

include_directories(${catkin_INCLUDE_DIRS})

add_library(AMBF_3DSlicer_Plugin SHARED ${HDR_FILES} ${SRC_FILES})
add_dependencies(AMBF_3DSlicer_Plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries (AMBF_3DSlicer_Plugin ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${catkin_LIBRARIES} ambf_framework chai3d )
set_property(TARGET AMBF_3DSlicer_Plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)
