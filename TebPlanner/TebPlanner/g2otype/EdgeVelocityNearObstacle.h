#include <BaseEdge.h>
#include <../Pose2D.h>
#include <../util.h>

namespace TebPlanner
{
class EdgeVelocityNearObstacle: public TebMultipleEdge<2,const Obstacle*>
{
public:
    EdgeVelocityNearObstacle()
    {
        _measurement = NULL;
    }
    void computeError()
    {
        ROS_ASSERT_MSG(config_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeVelocityNearObstacle");
        const VertexPose* point1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* point2 = static_cast<const VertexPose*>(_vertices[1]);
        const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

        const Eigen::Vector2d deltaS = point2->getPosition() - point1->getPosition();
        double dist = deltaS.norm();
        double theta1 = point1->getPose();
        double theta2 = point2->getPose();
        const double angle_diff = g2o::normalize_theta(theta2 - theta1);
        if (config_->trajectory.exact_arc_length && angle_diff != 0)
        {
            double radius =  dist/(2*sin(angle_diff/2));
            dist = fabs( angle_diff * radius ); // actual arg length!
        }
        double vel = dist / deltaT->getDiff();

        //vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction

        const double omega = angle_diff / deltaT->getDiff();
        const Obstacle* obstacle = _measurement;
        double dist_to_obstacle = obstacle->calculateMinDistance((*robot_model_), point1->getEstimate());

        double ratio;
        if (dist_to_obstacle < config_->obstacles.obstacle_proximity_lower_bound)
            ratio = 0;
        else if (dist_to_obstacle > config_->obstacles.obstacle_proximity_upper_bound)
            ratio = 1;
        else
            ratio = (dist_to_obstacle - config_->obstacles.obstacle_proximity_lower_bound) /
        (config_->obstacles.obstacle_proximity_upper_bound - config_->obstacles.obstacle_proximity_lower_bound);
        ratio *= config_->obstacles.obstacle_proximity_ratio_max_vel;

        const double max_vel_fwd = ratio * config_->robot.max_vel_x;
        const double max_omega = ratio * config_->robot.max_vel_theta;
        _error[0] = TebPlanner::LossFunction::rangeBound(vel, max_vel_fwd,-max_vel_fwd,0);
        _error[1] = TebPlanner::LossFunction::rangeBound(omega, max_omega,-max_omega,0);
    }

};
}

