

namespace TebPlanner
{
class VertexTimeDiff: public g2o::BaseVertex<1, double>
{
public:
    VertexTimeDiff(bool fixed=false)
    {
        _estimate = 0.01;
        setFixed(fixed);
    }
    VertexTimeDiff(double td,bool fixed=false)
    {
        _estimate = td;
        setFixed(fixed);
    }
    ~VertexTimeDiff(){}
    double& getDiff()
    {
        return _estimate;
    }
    virtual void oplusImpl(const double* update)
    {
        _estimate += *update;
    }
    virtual bool read(std::istream& is)
    {
        is >> _estimate;
        return true;
    }
    virtual bool write(std::ostream& os) const
    {
        os << estimate();
        return os.good();
    }

};
}
