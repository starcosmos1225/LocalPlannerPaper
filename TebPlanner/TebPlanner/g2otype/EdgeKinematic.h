#include <TebPlanner/g2otype/BaseEdge.h>

namespace TebPlanner
{
/**
 * @brief The EdgeKinematic class:This is a kinematic constraint of a differential drive model.
 *                                It should be noted that the kinematic constraints of the car model
 *                                are not this.
 * Since the kinematic constraints are only related to two adjacent local points, an edge only corresponds
 * to two vertices.
 */
class EdgeKinematicDiff:public TebBinaryEdge<2,double,VertexPose,VertexPose>
{
public:
    /**
     * @brief EdgeKinematic:default constructor.
     */
    EdgeKinematic(){}
    /**
     * @brief computeError:
     * Calculation formula reference:Trajectory modification considering dynamic constraints of autonomous robots
     * Different from the paper, in order to ensure the smoothness of the motion, it cannot be obtained by only
     * one constraint. Because this restriction may produce two opposite solutions, we choose a positive solution
     * through another constraint.
     */
    virtual void computeError()
    {
        const VertexPose* point1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* point2 = static_cast<const VertexPose*>(_vertices[1]);
        Eigen::Vector2d position1 = point1.getPosition();
        Eigen::Vector2d position2 = point2.getPosition();
        double theta1 = point1.getPose();
        double theta2 = point2.getPose();
        Eigen::Vector2d dist = position2-position1;
        // compute the kinematic error
        _error[0] = (cos(theta1)+cos(theta2))*dist[1]-(sin(theta1)+sin(theta2))*dist[0];
        // compute the direction error
        Eigen::Vector2d angle_vec( cos(theta1), sin(theta1));
        _error[1] = TebPlanner::LossFunction::lowerBound(dist.dot(angle_vec), 0,0);
    }

};
}
