TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/TebRos.cpp
INCLUDEPATH +=/opt/ros/kinetic/include
HEADERS += \
    TebPlanner/Configuration.h \
    TebPlanner/FootPrint.h \
    TebPlanner/Obstacle.h \
    TebPlanner/Planner.h \
    TebPlanner/Pose2D.h \
    TebPlanner/Teb.h \
    TebPlanner/TebRos.h \
    TebPlanner/type.h \
    TebPlanner/util.h \
    TebPlanner/g2otype/BaseEdge.h \
    TebPlanner/g2otype/EdgeAcceleration.h \
    TebPlanner/g2otype/EdgeKinematic.h \
    TebPlanner/g2otype/EdgeObstacle.h \
    TebPlanner/g2otype/EdgePoint.h \
    TebPlanner/g2otype/EdgeRotDirection.h \
    TebPlanner/g2otype/EdgeShortestPath.h \
    TebPlanner/g2otype/EdgeTimeDiff.h \
    TebPlanner/g2otype/EdgeVelocity.h \
    TebPlanner/g2otype/EdgeVelocityNearObstacle.h \
    TebPlanner/g2otype/VertexPose.h \
    TebPlanner/g2otype/VertexTimeDiff.h

