TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ../../src/graph_search.cpp \
    ../../src/homotopy_class_planner.cpp \
    ../../src/local_goal_planner.cpp \
    ../../src/obstacles.cpp \
    ../../src/optimal_planner.cpp \
    ../../src/recovery_behaviors.cpp \
    ../../src/teb_config.cpp \
    ../../src/teb_local_planner_ros.cpp \
    ../../src/test_optim_node.cpp \
    ../../src/timed_elastic_band.cpp \
    ../../src/visualization.cpp
INCLUDEPATH += ../../include
INCLUDEPATH += /opt/ros/melodic/include
INCLUDEPATH += /usr/include/eigen3
HEADERS += \
    ../../include/teb_local_planner/distance_calculations.h \
    ../../include/teb_local_planner/equivalence_relations.h \
    ../../include/teb_local_planner/g2o_types/base_teb_edges.h \
    ../../include/teb_local_planner/g2o_types/edge_acceleration.h \
    ../../include/teb_local_planner/g2o_types/edge_dynamic_obstacle.h \
    ../../include/teb_local_planner/g2o_types/edge_kinematics.h \
    ../../include/teb_local_planner/g2o_types/edge_leadhuman.h \
    ../../include/teb_local_planner/g2o_types/edge_local_obstacle.h \
    ../../include/teb_local_planner/g2o_types/edge_obstacle.h \
    ../../include/teb_local_planner/g2o_types/edge_plan.h \
    ../../include/teb_local_planner/g2o_types/edge_prefer_rotdir.h \
    ../../include/teb_local_planner/g2o_types/edge_shortest_path.h \
    ../../include/teb_local_planner/g2o_types/edge_time_optimal.h \
    ../../include/teb_local_planner/g2o_types/edge_velocity.h \
    ../../include/teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h \
    ../../include/teb_local_planner/g2o_types/edge_via_point.h \
    ../../include/teb_local_planner/g2o_types/penalties.h \
    ../../include/teb_local_planner/g2o_types/vertex_pose.h \
    ../../include/teb_local_planner/g2o_types/vertex_timediff.h \
    ../../include/teb_local_planner/graph_search.h \
    ../../include/teb_local_planner/h_signature.h \
    ../../include/teb_local_planner/homotopy_class_planner.h \
    ../../include/teb_local_planner/homotopy_class_planner.hpp \
    ../../include/teb_local_planner/local_goal_planner.h \
    ../../include/teb_local_planner/misc.h \
    ../../include/teb_local_planner/obstacles.h \
    ../../include/teb_local_planner/optimal_planner.h \
    ../../include/teb_local_planner/planner_interface.h \
    ../../include/teb_local_planner/pose_se2.h \
    ../../include/teb_local_planner/recovery_behaviors.h \
    ../../include/teb_local_planner/robot_footprint_model.h \
    ../../include/teb_local_planner/teb_config.h \
    ../../include/teb_local_planner/teb_local_planner_ros.h \
    ../../include/teb_local_planner/timed_elastic_band.h \
    ../../include/teb_local_planner/timed_elastic_band.hpp \
    ../../include/teb_local_planner/visualization.h \
    ../../include/teb_local_planner/visualization.hpp
