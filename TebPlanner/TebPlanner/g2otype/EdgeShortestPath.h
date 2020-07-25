#include <TebPlanner/g2otype/BaseEdge.h>

namespace TebPlanner
{
/**
 * @brief The EdgeShortestPath class:we optimize the distance and add a new choice "compute_arc_length_for_shortest_path".
 *                                   TEB compute the line distance, but we compute the arc distance for shortest.
 */
class EdgeShortestPath:public TebBinaryEdge<1,double,VertexPose,VertexPose>
{
public:
    /**
     * @brief EdgeShortestPath:the default constructor
     */
    EdgeShortestPath(){}
    /**
     * @brief computeError:compute the distance error.
     */
    void computeError()
    {
        VertexPose* point1 = static_cast<const VertexPose*>(_vertices[0]);
        VertexPose* point2 = static_cast<const VertexPose*>(_vertices[1]);
        Eigen::Vector2d position1 = point1->getPosition();
        Eigen::Vector2d position2 = point2->getPosition();
        double theta1 = point1->getPose();
        double theta2 = point2->getPose();
        double dist = (position2 - position1).norm();
        double angle_diff = theta2-theta1;
        if (config_->trajectory.compute_arc_length_for_shortest_path)
        {
            //compute the arc length instead the segment line length.
            if (angle_diff!=0)
            {
                const double radius =  dist/(2*sin(angle_diff/2));
                dist = fabs( angle_diff * radius );
            }
        }
        _error[0] = dist;
    }
};
}
