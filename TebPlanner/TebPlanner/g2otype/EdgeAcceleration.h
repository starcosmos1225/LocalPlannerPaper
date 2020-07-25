#include <TebPlanner/Pose2D.h>
#include <TebPlanner/util.h>
#include <TebPlanner/BaseEdge.h>
namespace TebPlanner
{
/**
 * @brief The EdgeAcceleration class:The acceleration-constrained edge is composed of three local
 *                                   point information and two time interval information. Together,
 *                                   they constrain the linear acceleration and angular acceleration.
 */
class EdgeAcceleration: public TebMultiEdge<2, double>
{
public:
    /**
     * @brief EdgeAcceleration:constructor.we call the resize(5) because there must be 5 vertexs link by the edge.
     */
    EdgeAcceleration()
    {
        this->resize(5);
    }
    /**
     * @brief computeError:compute the acceleration error.
     *                     For linear acceleration, we first calculate the straight line length of the two line
     *                     segments. Then according to the requirements, choose to calculate the arc length.
     *                     Divide by the time interval to get the speed. According to the acceleration formula:
     *                     acc = (v2-v1)*2/(dt1+dt2) to get the acceleration.
     * @return Cut off the actual acceleration and the required maximum acceleration.
     */
    virtual double computeError()
    {
        const VertexPose* point1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* point2 = static_cast<const VertexPose*>(_vertices[1]);
        const VertexPose* point3 = static_cast<const VertexPose*>(_vertices[2]);
        const VertexTimeDiff* dt1 = (*static_cast<const VertexTimeDiff*>(_vertices[3]));
        const VertexTimeDiff* dt2 = (*static_cast<const VertexTimeDiff*>(_vertices[4]));
        const double dist1 = (point2.getPosition()-point1.getPosition()).norm();
        const double dist2 = (point3.getPosition()-point2.getPosition()).norm();
        double angle_diff1 = g2o::normalize_theta(point2.getPose()-point1.getPose());
        double angle_diff2 = g2o::normalize_theta(point3.getPose()-point2.getPose());
        //compute the linear acceleration
        if (config_->trajectory.exact_arc_length)
        {
            //compute the arc length instead the segment line length.
            if (angle_diff1!=0)
            {
                const double radius =  dist1/(2*sin(angle_diff1/2));
                dist1 = fabs( angle_diff1 * radius );
            }
            if (angle_diff2!=0)
            {
                const double radius =  dist2/(2*sin(angle_diff2/2));
                dist2 = fabs( angle_diff2 * radius );
            }
        }
        double velocity1 = dist1/dt1->getDiff();
        double velocity2 = dits2/dt2->getDiff();
        const double linear_velocity_acceleration  = (velocity2-velocity1)*2/(dt1->getDiff() + dt2->getDiff());
        _error[0] = TebPlanner::LossFunction::rangeBound(linear_velocity_acceleration,
                                                         config_->robot.acc_lim_x,
                                                         -config_->robot.acc_lim_x,
                                                         config_->optim.penalty_epsilon);
        //compute the angle acceleration;
        double angle_velocity1 = angle_diff1/dt1->getDiff();
        double angle_velocity2 = angle_diff2/dt2->getDiff();
        double angle_acceleration = (angle_velocity2-angle_velocity1)*2/(dt1->getDiff()+dt2->getDiff());
        _error[1] = TebPlanner::LossFunction::rangeBound(angle_acceleration,config_->robot.acc_lim_theta,
                                                         -config_->robot.acc_lim_theta,config_->optim.penalty_epsilon);
    }
};

/**
 * @brief The EdgeAccelerationStart class:
 *        The acceleration and angular acceleration of the starting point are different from the intermediate
 *        point. One of its angular velocity and angular acceleration is given and does not need to be
 *        calculated. Therefore, 5 vertices are not needed, and only 3 vertices are needed.
 */
class EdgeAccelerationStart: public TebMultiEdge<2, double*>
{
public:
    /**
     * @brief EdgeAccelerationStart:constructor.we call the resize(3) because there must be 3 vertexs link by the edge.
     */
    EdgeAccelerationStart()
    {
        this->resize(3);
    }
    /**
     * @brief computeError:compute the acceleration error.
     *                     The velocity and angular velocity of the starting point are given and need to be
     *                     obtained by setVelocityAndAngleVel(). Therefore, we only need to calculate the
     *                     following velocity and angular velocity to get the linear acceleration and angular
     *                     acceleration.
     * @return Cut off the actual acceleration and the required maximum acceleration.
     */
    virtual double computeError()
    {
        const VertexPose* point2 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* point3 = static_cast<const VertexPose*>(_vertices[1]);
        const VertexTimeDiff* dt2 = (*static_cast<const VertexTimeDiff*>(_vertices[2]));
        const double dist2 = (point3.getPosition()-point2.getPosition()).norm();
        double angle_diff2 = g2o::normalize_theta(point3.getPose()-point2.getPose());
        //compute the linear acceleration
        if (config_->trajectory.exact_arc_length)
        {
            //compute the arc length instead the segment line length.
            if (angle_diff2!=0)
            {
                const double radius =  dist2/(2*sin(angle_diff2/2));
                dist2 = fabs( angle_diff2 * radius );
            }
        }
        double velocity1 = _measurement[0];
        double velocity2 = dits2/dt2->getDiff();
        const double linear_velocity_acceleration  = (velocity2-velocity1)/dt2->getDiff();
        _error[0] = TebPlanner::LossFunction::rangeBound(linear_velocity_acceleration,
                                                         config_->robot.acc_lim_x,
                                                         -config_->robot.acc_lim_x,
                                                         config_->optim.penalty_epsilon);
        //compute the angle acceleration;
        double angle_velocity1 = _measurement[1];
        double angle_velocity2 = angle_diff2/dt2->getDiff();
        double angle_acceleration = (angle_velocity2-angle_velocity1)/dt2->getDiff();
        _error[1] = TebPlanner::LossFunction::rangeBound(angle_acceleration,config_->robot.acc_lim_theta,
                                                         -config_->robot.acc_lim_theta,config_->optim.penalty_epsilon);
    }
    void setVelocityAndAngleVel(double *startVelocityAndAngleVel)
    {
        _measurement = startVelocityAndAngleVel;
    }
};
/**
 * @brief The EdgeAccelerationEnd class:
 *        The acceleration and angular acceleration of the ending point are different from the intermediate
 *        point. One of its angular velocity and angular acceleration is given and does not need to be
 *        calculated. Therefore, 5 vertices are not needed, and only 3 vertices are needed.
 */
class EdgeAccelerationEnd: public TebMultiEdge<2, double*>
{
public:
    /**
     * @brief EdgeAccelerationEnd:constructor.we call the resize(3) because there must be 3 vertexs link by the edge.
     */
    EdgeAccelerationEnd()
    {
        this->resize(3);
    }
    /**
     * @brief computeError:compute the acceleration error.
     *                     The velocity and angular velocity of the ending point are given and need to be
     *                     obtained by setVelocityAndAngleVel(). Therefore, we only need to calculate the
     *                     previous velocity and angular velocity to get the linear acceleration and angular
     *                     acceleration.
     * @return Cut off the actual acceleration and the required maximum acceleration.
     */
    virtual double computeError()
    {
        const VertexPose* point1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* point2 = static_cast<const VertexPose*>(_vertices[1]);
        const VertexTimeDiff* dt1 = (*static_cast<const VertexTimeDiff*>(_vertices[2]));
        const double dist1 = (point2.getPosition()-point1.getPosition()).norm();
        double angle_diff1 = g2o::normalize_theta(point2.getPose()-point1.getPose());
        //compute the linear acceleration
        if (config_->trajectory.exact_arc_length)
        {
            //compute the arc length instead the segment line length.
            if (angle_diff1!=0)
            {
                const double radius =  dist1/(2*sin(angle_diff1/2));
                dist1 = fabs( angle_diff1 * radius );
            }
        }
        double velocity2 = _measurement[0];
        double velocity1 = dits1/dt1->getDiff();
        const double linear_velocity_acceleration  = (velocity2-velocity1)/dt1->getDiff();
        _error[0] = TebPlanner::LossFunction::rangeBound(linear_velocity_acceleration,
                                                         config_->robot.acc_lim_x,
                                                         -config_->robot.acc_lim_x,
                                                         config_->optim.penalty_epsilon);
        //compute the angle acceleration;
        double angle_velocity2 = _measurement[1];
        double angle_velocity1 = angle_diff1/dt1->getDiff();
        double angle_acceleration = (angle_velocity2-angle_velocity1)/dt1->getDiff();
        _error[1] = TebPlanner::LossFunction::rangeBound(angle_acceleration,config_->robot.acc_lim_theta,
                                                         -config_->robot.acc_lim_theta,config_->optim.penalty_epsilon);
    }
    void setVelocityAndAngleVel(double *endVelocityAndAngleVel)
    {
        _measurement = endVelocityAndAngleVel;
    }
};
}
