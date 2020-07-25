#include <TebPlanner/g2otype/BaseEdge.h>

namespace TebPlanner
{
/**
 * @brief The EdgeVelocity class:constrain the linear velocity and angle velocity
 */
class EdgeVelocity: public TebMultipleEdge<2,double>
{
public:
    /**
     * @brief EdgeVelocity:the default constructor
     */
    EdgeVelocity(){}
    /**
     * @brief computeError:we compute the velocity and angle velocity, then check whether the value exceed
     *                     limit.
     */
    void computeError()
    {
        const VertexPose* point1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* point2 = static_cast<const VertexPose*>(_vertices[1]);
        const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

        const Eigen::Vector2d deltaS = point2.getPosition() - point1.getPosition();

        double dist = deltaS.norm();
        const double angle_diff = g2o::normalize_theta(point2->getPose() - point1->getPose());
        if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
        {
            // actual arc length!
            double radius =  dist/(2*sin(angle_diff/2));
            dist = fabs( angle_diff * radius );
        }
        double vel = dist / deltaT->getDiff();

    //     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
    //    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction

        const double omega = angle_diff / deltaT->getDiff();

        _error[0] = TebPlanner::LossFunction::rangeBound(vel, cfg_->robot.max_vel_x, -cfg_->robot.max_vel_x_backwards,cfg_->optim.penalty_epsilon);
        _error[1] = TebPlanner::LossFunction::rangeBound(omega, cfg_->robot.max_vel_theta,-cfg_->robot.max_vel_theta, cfg_->optim.penalty_epsilon);

        ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
    }
};
}
