cmake_minimum_required(VERSION 2.8.3)
project(bb_mpu9150)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
   INCLUDE_DIRS 
#  LIBRARIES mpu9150
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Alternatively we can use the CMakeLists.txt of the subdirs
#add_subdirectory(src/linux-mpu9150)

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/linux-mpu9150
  src/linux-mpu9150/eMPL
  src/linux-mpu9150/glue
  src/linux-mpu9150/mpu9150
)

#link_directories()


# Test to verify simple ROS node
add_executable(random src/random.cpp)
target_link_libraries(random ${catkin_LIBRARIES} )

# The MPU9150 node
# Adds flags to the compiler command line for sources in the current directory and below.
add_definitions(-DEMPL_TARGET_LINUX -DMPU9150 -DAK8975_SECONDARY)
add_executable(mpu9150_node src/mpu9150_node.cpp)
add_library(mpu9150 SHARED src/linux-mpu9150/mpu9150/mpu9150.c)
add_library(linux_glue SHARED src/linux-mpu9150/glue/linux_glue.c)
add_library(vector3d SHARED src/linux-mpu9150/mpu9150/vector3d.c)
add_library(quaternion SHARED src/linux-mpu9150/mpu9150/quaternion.c)
add_library(inv_mpu SHARED src/linux-mpu9150/eMPL/inv_mpu.c)
add_library(inv_mpu_dmp_motion_driver SHARED src/linux-mpu9150/eMPL/inv_mpu_dmp_motion_driver.c)
target_link_libraries(mpu9150_node linux_glue mpu9150 inv_mpu quaternion inv_mpu_dmp_motion_driver vector3d ${catkin_LIBRARIES})
#target_link_libraries(mpu9150_node ${catkin_LIBRARIES})

# imu utility
#add_executable(imu src/linux-mpu9150/imu.c)
#target_link_libraries(imu vector3d linux_glue mpu9150 inv_mpu  quaternion inv_mpu_dmp_motion_driver)

# imucal utility (calibration utility)
#add_executable(imucal src/linux-mpu9150/imucal.c)
#target_link_libraries(imucal linux_glue mpu9150 inv_mpu quaternion inv_mpu_dmp_motion_driver vector3d ${catkin_LIBRARIES})

install(TARGETS mpu9150_node linux_glue mpu9150 inv_mpu quaternion inv_mpu_dmp_motion_driver vector3d
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES src/accelcal.txt src/magcal.txt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY src/linux-mpu9150 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
