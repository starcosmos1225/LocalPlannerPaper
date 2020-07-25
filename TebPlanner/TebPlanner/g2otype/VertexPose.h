#include <../Pose2D.h>

namespace TebPlanner
{
class VertexPose:public g2o::BaseVertex<3, Pose2D>
{
public:
    VertexPose(bool fixed = false)
    {
        _estimate = 0;
        setFixed(fixed);
    }
    VertexPose(const Pose2D& pose,bool fixed = false)
    {
        _estimate = pose;
        setFixed(fixed);
    }
    VertexPose(const Eigen::Vector2d& position,double theta,bool fixed = false)
    {
        _estimate.setPosition(position);
        _estimate.setPose(theta);
        setFixed(fixed);
    }
    VertexPose(double x,double y,double theta,bool fixed=false)
    {
        _estimate.setPosition(x,y);
        _estimate.setPose(theta);
        setFixed(fixed);
    }
    ~VertexPose(){}
    Eigen::Vetor2d& getPosition()
    {
        return _estimate.getPosition();
    }
    double getPose()
    {
        return _estimate.getPose();
    }
    Pose2D getEstimate()
    {
        return _estimate;
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate.plus(update);
    }
    virtual bool read(std::istream& is)
    {
        is >> _estimate.x() >> _estimate.y() >> _estimate.theta();
        return true;
    }
    virtual bool write(std::ostream& os) const
    {
        os << _estimate.x() << " " << _estimate.y() << _estimate.theta();
        return os.good();
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
