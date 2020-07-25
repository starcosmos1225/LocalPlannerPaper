#pragma once
#include <BaseEdge.h>
#include <../util.h>
namespace TebPlanner
{
class EdgePoint:public TebUnaryEdge<1,const Eigen::Vector2d*,VertexPose>
{
public:
    EdgePoint()
    {
        _measurement = NULL;
    }
    void computeError()
    {
        ROS_ASSERT_MSG(config_ && _measurement, "You must call setTebConfig(), setPoint() on EdgePoint()");
        const VertexPose* point = static_cast<const VertexPose*>(_vertices[0]);

        double dist = (point->getPosition() - *_measurement).norm();
        _error[0] = TebPlanner::LossFunction::rangeBound(dist,config_->AttractPoint.max_dist,config_->AttractPoint.min_dist,config_->AttractPoint.noise);
    }
    void setPoint(const Eigen::Vector2d* point)
    {
        _measurement = point;
    }
};
}
