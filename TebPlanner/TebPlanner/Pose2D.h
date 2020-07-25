#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Core>
namespace TebPlanner
{
/**
 * @brief The Pose2D class:struct for position and pose in 2D.
 *        the position has two varible:x y
 *        the pose has one varible: theta
 */
class Pose2D
{
public:
    /**
     * @brief Pose2D:default constructor
     */
    Pose2D();
    /**
     * @brief Pose2D:constructor with position
     * @param x:the x of position
     * @param y:the y of position
     * @param pose:the pose angle along x axis
     */
    Pose2D(double x, double y, double pose=0.0);
    /**
     * @brief Pose2D:another way to construct the class
     * @param position: vector2d of position
     * @param pose:the pose angle along x axis
     */
    Pose2D(Eigen::Vector2d position, double pose=0.0);
    Pose2D(geometry_msgs::PoseStamped& pose);
    /**
     * @brief ~Pose2D:default destructor
     */
    ~Pose2D();
    /**
     * @brief setPose: set the pose value
     * @return
     */
    void setPose(double pose);
    /**
     * @brief getPose: get the pose value
     * @return pose
     */
    double getPose();
    /**
     * @brief setPosition: another way to set position
     * @param position:vector2d of position
     */
    void setPosition(Eigen::Vector2d position);
    /**
     * @brief setPosition: set the position
     * @param x:position.x
     * @param y:position.y
     */
    void setPosition(double x, double y);
    /**
     * @brief getPosition: get the position
     * @return vector2d
     */
    Eigen::Vector2d getPosition();
    /**
     * @brief x: get the X of position
     * @return x
     */
    double& x();
    /**
     * @brief y: get the Y of position
     * @return y
     */
    double& y();
    /**
     * @brief theta: get the pose
     * @return theta
     */
    double& theta();

private:
    Eigen::Vector2d position_;//!<position of the local point
    double pose_;//!<angle of the local point
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
