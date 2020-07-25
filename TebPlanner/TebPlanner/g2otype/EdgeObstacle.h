#include <TebPlanner/g2otype/BaseEdge.h>
#include <TebPlanner/Obstacle.h>
#include <TebPlanner/Pose2D.h>
namespace TebPlanner
{
class EdgeObstacle: public TebUnaryEdge<1,const Obstacle*,VertexPose>
{
public:
    EdgeObstacle()
    {
        _measurement = NULL;
    }
    void computeError()
    {
        ROS_ASSERT_MSG(_measurement&&robot_model_&&config_,"No Obstacle or RobotModel or Config setted");
        const VertexPose* point = static_cast<const VertexPose*>(_vertices[0]);
        const Obstacle* obstacle = _measurement;
        double dist = obstacle->calculateMinDistance((*robot_model_), point->getEstimate());

        // Original obstacle cost.
        _error[0] = TebPlanner::LossFunction::lowerBound(dist, config_->obstacles.min_obstacle_dist, config_->optim.penalty_epsilon);

        if (config_->optim.obstacle_cost_exponent != 1.0 && config_->obstacles.min_obstacle_dist > 0.0)
        {
          // Optional non-linear cost. Note the max cost (before weighting) is
          // the same as the straight line version and that all other costs are
          // below the straight line (for positive exponent), so it may be
          // necessary to increase weight_obstacle and/or the inflation_weight
          // when using larger exponents.
          _error[0] = config_->obstacles.min_obstacle_dist * std::pow(_error[0] / config_->obstacles.min_obstacle_dist, config_->optim.obstacle_cost_exponent);
        }

    }
    void setObstacle(const Obstacle* obstacle)
    {
        _measurement = obstacle;
    }
    void setRobotModel(const FootPrint* robot_model)
    {
        robot_model_ = robot_model;
    }

protected:
    FootPrint* robot_model_;//!<the robot's footprint model

};
}
