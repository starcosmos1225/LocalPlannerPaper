#include "type.h"

namespace TebPlanner
{
/**
 * @brief The FootPrint class:This class defines the shape and size of robot
 */
class FootPrint
{
public:
    /**
     * @brief FootPrint:the default constructor
     */
    FootPrint(){is_init_ = false;}
    /**
     * @brief FootPrint:another constructor to initialize the vertexs and radius
     * @param vertexs: the vertexs which construct the footprint's shape
     * @param radius:If the footprint is a circle,the radius can be useful. Otherwise the radius is -1.0.
     */
    FootPrint(std::vector<Eigen::Vector2d> &vertexs,double radius = -1.0)
    {
        initialize(vertexs,radius);
    }
    /**
     * @brief ~FootPrint:the default destructor
     */
    ~FootPrint(){}
    /**
     * @brief initialize:init the footprint, set its shape and radius
     * @param vertexs: the vertexs which construct the footprint's shape
     * @param radius:If the footprint is a circle,the radius can be useful. Otherwise the radius is -1.0.
     */
    void initialize(std::vector<Eigen::Vector2d> &vertexs,double radius = -1.0)
    {
        if (is_init_)
        {
            ROS_WARN("REINIT:ReInitialize FootPrint");
        }
        radius_ = radius;
        vertexs_ = vertexs;
        center_ = TebPlanner::computeCenter(vertexs_);
        is_init_ = true;
    }
    /**
     * @brief setWorldPose:Due to computational needs, we need to get the pose of the robot model in the real
     *                     world, so use this function to get the coordinates of the model points in the real
     *                     world. The actual execution function is transform2World
     * @param pose:Real-world robot pose center
     * @param vertexs[in,out]:the shape we need to get
     */
    void setWorldPose(const Pose2D& pose,std::vector<Eigen::Vector2d> &vertexs)
    {
        transform2World(pose,vertexs);
    }
    /**
     * @brief setVertex:we can set the robot's shape manually.
     * @param vertexs:the shape form with vertexs
     */
    void setVertex(std::vector<Eigen::Vector2d> &vertexs)
    {
        vertexs_ = vertexs;
        center_ = TebPlanner::computeCenter(vertexs_);
    }
    /**
     * @brief getVertex get the shape
     * @return
     */
    std::vector<Eigen::Vector2d> getVertex(){return vertexs_;}
    /**
     * @brief setCenter set center manually
     * @param center:the 2d location
     */
    void setCenter(Eigen::Vector2d &center){center_=center;}
    /**
     * @brief getCenter:get the center location
     * @return center
     */
    Eigen::Vector2d getCenter(){return center_;}
    /**
     * @brief setRadius:set the radius manually
     * @param radius
     */
    void setRadius(double radius){radius_=radius;}
    /**
     * @brief getRadius:get the radius
     * @return radius
     */
    double getRadius(){return radius_;}
    /**
     * @brief getType:for the FootPrint I do not use the virtual class to make different type of shape,
     *                but check the shape type by function
     * @return type Id
     */
    int getType()
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return -1;
        }
        if (radius>0)
        {
            return TebPlanner::CIRCLETYPE;
        }
        else if(vertexs.size()==1)
        {
            return TebPlanner::POINTTYPE;
        }else if (vertexs.size()==2)
        {
            return TebPlanner::LINETYPE;
        }else if (vertexs.size()>2)
        {
            return TebPlanner::POLYGONTYPE;
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
    /**
     * @brief IsInner check whether a point in the footprint
     * @param vertex:point location
     * @param minDist:for line and point we check the distance
     * @return true if in the footprint, otherwise false
     */
    bool IsInner(const Eigen::Vector2d &vertex, double minDist = 0.0)
    {
        int type = getType();
        if (type ==TebPlanner::POLYGONTYPE)
            return TebPlanner::isPointInPolygon(vertexs_,vertex);
        if (type ==TebPlanner::CIRCLETYPE)
            return TebPlanner::isPointInCircle(center_,radius,vertex);
        if (type ==TebPlanner::POINTTYPE)
            return checkCollisionPoints(vertexs_[0], vertex,minDist);
        if (type ==TebPlanner::LINETYPE)
            return checkCollisionPointLine(vertex,vertexs_,minDist);
        ROS_ERROR("TYPEERROR:Unkown Type Occur");
        return false;
    }
protected:
    /**
     * @brief transform2World:When robot move,the vertexs of its shape are not in the origin position.
     *                        It will change to a new position.We use this function to compute the current
     *                        position of the vertexs but do not change the original one.
     * @param pose:the robot's current pose
     * @param vertexs[in,out]:the current positions we need to compute.
     */
    void transform2World(const Pose2D& pose,std::vector<Eigen::Vector2d> &vertexs)
    {
        //TODO
    }
private:
    std::vector<Eigen::Vector2d> vertexs_;
    Eigen::Vector2d center_;
    double radius_;
    bool is_init_;
};
}
