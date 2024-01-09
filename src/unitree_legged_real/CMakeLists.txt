cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    unitree_legged_msgs
    tf
)
find_package(pinocchio REQUIRED)
find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)
include_directories(/usr/local/include/osqp)
find_package(OsqpEigen REQUIRED)
catkin_package()

set(MUJOCO_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco-2.3.5)
message("MuJoCo path: " ${MUJOCO_PATH})


include_directories(
    include
    ${MUJOCO_PATH}/include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${MUJOCO_PATH}/bin
)

include_directories(${MUJOCO_PATH}/include/mujoco)
file(GLOB MJ_LIB ${MUJOCO_PATH}/lib/libmujoco.so)

add_library(hardware_a1_lib
        src/exe/A1_Interface.h
        src/exe/A1_Interface.cpp
        src/exe/A1_dynamics.h
        src/exe/A1_dynamics.cpp
        src/exe/A1Params.h
        src/exe/A1_state_estimation.cpp
        src/exe/A1_state_estimation.h
        src/exe/Utils.h
        src/exe/Utils.cpp
        src/exe/A1_QP_MPC.h
        src/exe/A1_QP_MPC.cpp
        src/exe/A1_control.h
        src/exe/A1_control.cpp
        src/exe/Bezier_curve.h
        src/exe/Bezier_curve.cpp
        # src/exe/A1_Mujoco.cpp
        # src/exe/A1_Mujoco.h
        src/exe/A1_Gazebo.cpp
        src/exe/A1_Gazebo.h
)

add_library(realsense_a1_lib
        src/exe/A1_dynamics.h
        src/exe/A1_dynamics.cpp
        src/exe/A1Params.h
        src/exe/A1_state_estimation.cpp
        src/exe/A1_state_estimation.h   
        src/exe/A1_realsense.cpp
        src/exe/A1_realsense.h
        src/exe/Utils.h
        src/exe/Utils.cpp
        src/exe/Bezier_curve.h
        src/exe/Bezier_curve.cpp
)       

add_library(a1_dynamics_lib
        src/exe/A1_dynamics_pino.h
        src/exe/A1_dynamics_pino.cpp
)


set(CMAKE_CXX_FLAGS "-O3")

if( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_1")
    include_directories($ENV{ALIENGO_SDK_PATH}/include)
    link_directories($ENV{ALIENGO_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libaliengo_comm.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

    add_definitions(-DSDK3_1)

    add_executable(lcm_server_3_1 $ENV{ALIENGO_SDK_PATH}/examples/lcm_server.cpp)
    target_link_libraries(lcm_server_3_1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
    add_dependencies(lcm_server_3_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

elseif( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_2")
    include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include)
    link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

    add_definitions(-DSDK3_2)

    add_executable(lcm_server_3_2 $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server.cpp)
    target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
    add_dependencies(lcm_server_3_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(test_mpc src/exe/QP_mpc_test.cpp)
target_link_libraries(test_mpc
        hardware_a1_lib
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        OsqpEigen::OsqpEigen
        )

add_executable(test_bezier src/exe/test_bezier.cpp)
target_link_libraries(test_bezier
        hardware_a1_lib
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        OsqpEigen::OsqpEigen
        )

add_executable(position_lcm src/exe/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(position_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(velocity_lcm src/exe/velocity_mode.cpp)
# target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(velocity_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(torque_lcm src/exe/torque_mode.cpp)
# target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(walk_lcm src/exe/walk_mode.cpp)
# target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hardware_lcm src/exe/robot_hardware.cpp)
target_link_libraries(hardware_lcm hardware_a1_lib a1_dynamics_lib ${EXTRA_LIBS} ${catkin_LIBRARIES} pinocchio::pinocchio OsqpEigen::OsqpEigen)
add_dependencies(hardware_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dynamic_test src/exe/test_dynamics.cpp)
target_link_libraries(dynamic_test a1_dynamics_lib ${EXTRA_LIBS} ${catkin_LIBRARIES} pinocchio::pinocchio)
# add_dependencies(dynamic_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(a1_gazebo src/exe/robot_gazebo.cpp)
target_link_libraries(a1_gazebo hardware_a1_lib Eigen3::Eigen a1_dynamics_lib ${MJ_LIB} ${OpenMP} ${EXTRA_LIBS} ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)


add_executable(a1_realsense src/exe/Main_Realsense.cpp)
target_link_libraries(a1_realsense realsense_a1_lib a1_dynamics_lib ${EXTRA_LIBS} ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)
# add_executable(a1_mujoco src/exe/MainMujoco.cpp)
# target_link_libraries(a1_mujoco hardware_a1_lib Eigen3::Eigen a1_dynamics_lib ${MJ_LIB} ${OpenMP} ${MUJOCO_PATH}  ${EXTRA_LIBS} ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

# add_executable(mujoco_recv src/exe/Mujoco_Recv_Visual.cpp)
# target_link_libraries(mujoco_recv hardware_a1_lib Eigen3::Eigen a1_dynamics_lib ${MJ_LIB} ${OpenMP} ${MUJOCO_PATH}  ${EXTRA_LIBS} ${catkin_LIBRARIES} pinocchio::pinocchio OsqpEigen::OsqpEigen glfw)