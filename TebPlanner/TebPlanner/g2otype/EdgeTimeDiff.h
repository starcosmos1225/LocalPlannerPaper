#include <TebPlanner/g2otype/BaseEdge.h>

namespace TebPlanner
{
/**
 * @brief The EdgeTimeDiff class:make constrain for fastest time
 */
class EdgeTimeDiff: public BaseUnaryEdge<1,double,VertexTimeDiff>
{
public:
    /**
     * @brief EdgeTimeDiff:default constructor
     */
    EdgeTimeDiff(){}
    /**
     * @brief computeError:our target is to make sum of time diff minimum
     */
    void computeError()
    {
        VertexTimeDiff *timediff = static_cast<const VertexTimeDiff*>(_vertices[0]);
        _error = timediff->getDiff();
    }
};
}
