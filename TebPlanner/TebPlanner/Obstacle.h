#include "util.h"

namespace TebPlanner
{
/**
 * @brief The Obstacle class:virtual class for obstacle,the implement will in PointObstacle,LineObsatcle,
 *                           CircleObstacle and PolygonObstacle
 */
class Obstacle
{
public:
    /**
     * @brief Obstacle:default constructor.
     */
    Obstacle()
    {
        is_init_ = false;
    }

    /**
     * @brief Obstacle:constructor with point and radius
     * @param points: containner of points
     * @param radius: for circle
     */
    Obstacle(std::vector<Eigen::Vector2d> points, double radius = -1.0)
    {
        initialize(points,radius);
    }

    /**
     * @brief ~Obstacle:default destructor.
     */
    virtual ~Obstacle();
    /**
     * @brief initialize:initialize the obstacle
     * @param points:the points construct the obstacle
     * @param radius:the radius for circle. If radius = -1.0, the obstacle is not a CircleObstacle
     */
    virtual void initialize(const std::vector<Eigen::Vector2d>& points, double radius = -1.0)
    {
        if (is_init_)
        {
            ROS_WARN("REINIT:ReInitialize Obstacles");
        }
        radius_ = radius;
        vertexs_ = points;
        center_ = TebPlanner::computeCenter(vertexs_);
        is_init_ = true;
    }

    /**
     * @brief checkCollision:check collision between footprint and PointObstacle
     * @param footprint:robot model
     * @param point:The pose of the robot model in the real world
     * @param minDistance:If the distance between footprint and obstacle < minDistance, the collision occur.
     *                    Otherwise safe.
     * @return True if collision occur,Otherwise false
     */
    virtual bool checkCollision(FootPrint &footprint,const Pose2D &point, double minDistance = 0)=0;
    /**
     * @brief calculateMinDistance:compute min distance between obstacle and footprint
     * @param footprint:the robot model
     * @param point:the robot current pose
     * @return
     */
    virtual double calculateMinDistance(const FootPrint &footprint,const Pose2D &point)=0;

protected://!<protected varible
    std::vector<Eigen::Vector2d> vertexs_;//!<the point form the obstacle
    Eigen::Vector2d center_;//!<the center point
    double radius_;//!<radius for circle
    bool is_init_;//!<flag of initialize

};
/**
 * @brief The PointObstacle class:obstacle present by a point
 */
class PointObstacle:public Obstacle
{
public:
    /**
     * @brief PointObstacle:default constructor
     */
    PointObstacle(){}
    /**
     * @brief ~PointObstacle:default destructor
     */
    ~PointObstacle(){}
    /**
     * @brief checkCollision:check collision between footprint and PointObstacle
     * @param footprint:robot model
     * @param point:The pose of the robot model in the real world
     * @param minDistance:If the distance between footprint and obstacle < minDistance, the collision occur.
     *                    Otherwise safe.
     * @return True if collision occur,Otherwise false
     */
    virtual bool checkCollision(FootPrint &footprint,const Pose2D &point,double minDistance = 0)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setWorldPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::checkCollisionPoints(vertexs_[0],footprint_vertex, minDistance);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::checkCollisionPointCircle(vertexs_[0], footprint.getCenter(), footprint_radius, minDistance);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::checkCollisionPointLine(vertexs_[0],footprint_vertex, minDistance);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::checkCollisionPointPolygon(vertexs_[0],footprint_vertex, minDistance);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
    /**
     * @brief calculateMinDistance:compute min distance between obstacle and footprint
     * @param footprint:the robot model
     * @param point:the robot current pose
     * @return
     */
    virtual double calculateMinDistance(const FootPrint &footprint,const Pose2D &point)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::computeDistancePoints(vertexs_[0],footprint_vertex);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::computeDistancePointCircle(vertexs_[0], point.getPosition(), footprint_radius);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::computeDistancePointLine(vertexs_[0],footprint_vertex);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::computeDistancePointPolygon(vertexs_[0],footprint_vertex);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
};
class LineObstacle:public Obstacle
{
public:
    /**
     * @brief LineObstacle:default constructor
     */
    LineObstacle(){}
    /**
     * @brief ~LineObstacle:default destructor
     */
    ~LineObstacle(){}
    /**
     * @brief checkCollision:check collision between footprint and PointObstacle
     * @param footprint:robot model
     * @param point:The pose of the robot model in the real world
     * @param minDistance:If the distance between footprint and obstacle < minDistance, the collision occur.
     *                    Otherwise safe.
     * @return True if collision occur,Otherwise false
     */
    virtual bool checkCollision(FootPrint &footprint,const Pose2D &point, double minDistance = 0)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setWorldPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::checkCollisionPointLine(footprint_vertex[0],vertexs_, minDistance);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::checkCollisionLineCircle(vertexs_, footprint.getCenter(), footprint_radius, minDistance);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::checkCollisionLines(vertexs_,footprint_vertex, minDistance);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::checkCollisionLinePolygon(vertexs_,footprint_vertex, minDistance);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
    /**
     * @brief calculateMinDistance:compute min distance between obstacle and footprint
     * @param footprint:the robot model
     * @param point:the robot current pose
     * @return
     */
    virtual double calculateMinDistance(const FootPrint &footprint,const Pose2D &point)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::computeDistancePointLine(footprint_vertex[0],vertexs_);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::computeDistanceLineCircle(vertexs_, point.getPosition(), footprint_radius);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::computeDistanceLines(vertexs_,footprint_vertex);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::computeDistanceLinePolygon(vertexs_,footprint_vertex);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
};
class CircleObstacle:public Obstacle
{
public:
    /**
     * @brief CircleObstacle:default constructor
     */
    CircleObstacle(){}
    /**
     * @brief ~CircleObstacle:default destructor
     */
    ~CircleObstacle(){}
    /**
     * @brief checkCollision:check collision between footprint and PointObstacle
     * @param footprint:robot model
     * @param point:The pose of the robot model in the real world
     * @param minDistance:If the distance between footprint and obstacle < minDistance, the collision occur.
     *                    Otherwise safe.
     * @return True if collision occur,Otherwise false
     */
    virtual bool checkCollision(FootPrint &footprint,const Pose2D &point, double minDistance = 0)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setWorldPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::checkCollisionPointCircle(footprint_vertex[0],center_, radius_, minDistance);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::checkCollisionLineCircle(footprint_vertex, center_, radius_, minDistance);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::checkCollisionCircles(center_,radius_,footprint.getCenter(),footprint_radius, minDistance);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::checkCollisionCirclePolygon(center_,radius_,footprint_vertex, minDistance);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
    /**
     * @brief calculateMinDistance:compute min distance between obstacle and footprint
     * @param footprint:the robot model
     * @param point:the robot current pose
     * @return
     */
    virtual double calculateMinDistance(const FootPrint &footprint,const Pose2D &point)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::computeDistancePointCircle(footprint_vertex[0],center_, radius_);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::computeDistanceLineCircle(footprint_vertex, center_, radius_);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::computeDistanceCircles(center_,radius_,point.getPosition(),footprint_radius);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::computeDistanceCirclePolygon(center_,radius_,footprint_vertex);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }

};
class PolygonObstacle:public Obstacle
{
public:
    PolygonObstacle(){}
    ~PolygonObstacle(){}
    /**
     * @brief checkCollision:check collision between footprint and PointObstacle
     * @param footprint:robot model
     * @param point:The pose of the robot model in the real world
     * @param minDistance:If the distance between footprint and obstacle < minDistance, the collision occur.
     *                    Otherwise safe.
     * @return True if collision occur,Otherwise false
     */
    virtual bool checkCollision(FootPrint &footprint,const Pose2D &point, double minDistance = 0)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setWorldPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::checkCollisionPointPolygon(footprint_vertex[0],vertexs_, minDistance);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::checkCollisionCirclePolygon(footprint.getCenter(),footprint_radius,
                                                           vertexs_, minDistance);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::checkCollisionLinePolygon(footprint_vertex,vertexs_, minDistance);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::checkCollisionPolygons(footprint_vertex,vertexs_, minDistance);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
    /**
     * @brief calculateMinDistance:compute min distance between obstacle and footprint
     * @param footprint:the robot model
     * @param point:the robot current pose
     * @return
     */
    virtual double calculateMinDistance(const FootPrint &footprint,const Pose2D &point)
    {
        if (!is_init_)
        {
            ROS_ERROR("INITERROR:Function Excute Before Initialize");
            return false;
        }
        std::vector<Eigen::Vector2d > footprint_vertex;
        double footprint_radius = footprint->getRadius();
        footprint->setWorldPose((*point),footprint_vertex);
        int footprint_type = footprint.getType();

        if (footprint_type == TebPlanner::POINTTYPE)
        {
            return TebPlanner::computeDistancePointPolygon(footprint_vertex[0],vertexs_);
        }else if (footprint_type == TebPlanner::CIRCLETYPE)
        {
            return TebPlanner::computeDistanceCirclePolygon(point.getPosition(),footprint_radius,
                                                           vertexs_);
        }else if (footprint_type == TebPlanner::LINETYPE)
        {
            return TebPlanner::computeDistanceLinePolygon(footprint_vertex,vertexs_);
        }else if (footprint_type == TebPlanner::POLYGONTYPE)
        {
            return TebPlanner::computeDistancePolygons(footprint_vertex,vertexs_);
        }else
        {
            ROS_ERROR("TYPEERROR:Unkown Type Occur");
        }
    }
};
}
