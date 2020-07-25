#include <TebPlanner/g2otype/BaseEdge.h>

namespace TebPlanner
{
/**
 * @brief The EdgeRotDirection class:The constrain is not in the paper,but we can set the constrain to set prefer
 *                                   rotate,left or right.
 *                                   the measurement 1=left and -1=right.
 */
class EdgeRotDirection:public TebBinaryEdge<1,double,VertexPose,VertexPose>
{
public:
    /**
     * @brief EdgeRotDirection:default constructor,we set left rotate prefer.
     */
    EdgeRotDirection()
    {
        _measurement = 1;
    }
    /**
     * @brief computeError:the error compute to cut off when rotate left but prefer right or rotate right but
     *                     prefer left.
     */
    void computeError()
    {
        const VertexPose* point1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* point2 = static_cast<const VertexPose*>(_vertices[1]);
        double theta1 = point1->getPose();
        double theta2 = point2->getPose();
        _error[0] = TebPlanner::LossFunction::lowerBound( _measurement*g2o::normalize_theta(theta2-theta1) , 0, 0);
    }
    /**
     * @brief setPreferRot:set the prefer rotate.
     * @param rotate:1 = left -1 = right.
     */
    void setPreferRot(double rotate)
    {
        _measurement = rotate;
    }
    /**
     * @brief setLeft:set the left prefer
     */
    void setLeft()
    {
        _measurement = 1.0;
    }
    /**
     * @brief setRight:set the right prefer
     */
    void setRight()
    {
        _measurement = -1.0;
    }

};

}
