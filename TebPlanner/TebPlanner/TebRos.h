/***************
 * author:huxingyu
 *        ZJU CS
 * *************/
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <TebPlanner/FootPrint.h>
#include <TebPlanner/Planner.h>
#include <TebPlanner/Configuration.h>
#include <TebPlanner/Pose2D.h>
#include <TebPlanner/Obstacle.h>

#include <tf2/utils.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

#include <eigen3/Eigen/Core>
#include <g2o/stuff/misc.h>

#include <mbf_msgs/ExePathResult.h>

namespace TebPlanner
{
/*teb_ros:
 * It is an interface class with ROS, which is used to receive information
 * from ROS and transmit data according to the specified format.
 */
class TebRos: public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController
{
public:
    /**
     * @brief default constructor
     */
    TebRos();
    /**
     * @brief default destructor
     */
    ~TebRos();
    /**
     * @brief For initialization functions inherited from the parent class, the parameters are
     *        required by the parent class. Used to initialize local planning.
     * @param name: The name of the instance
     * @param tf: A pointer to a transform listener
     * @param costmap_ros:The costmap pointer passed from move_base can obtain costmap information from it.
     */
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
    /**
     * @brief setPlan:Set the plan that the local planner is following.
     * @param plan:The plan come from move_base, maybe the global plan.
     * @return true for success set planner, otherwise false.
     */
    bool setPlan(const std::vector< geometry_msgs::PoseStamped > &global_plan);
    /**
     * @brief computeVelocityCommands:Given the current position, orientation, and velocity of the robot,
     *                                compute velocity commands to send to the base.
     * @param cmd_vel:the msg for velocity and angular velocity. It is the return so we must change the cmd_vel to
     *                offer current velocity and angular velocity.
     * @return true for success get a Valid values, otherwise false.
     */
    bool computeVelocityCommands(geometry_msgs::TwistStamped &cmd_vel);
    /**
     * @brief Check if the goal pose has been achieved by the local planner.
     * @return true for reach, otherwise false.
     */
    bool isGoalReached();
    /**
     * @brief cancel:Cancel the plan. It is no need for local planner to implement.
     * @return always return false.
     */
    bool cancel();
protected://!<protected function
    /**
     * @brief updateAndPrunePlan:With the movement of robot, the path of global path planning will
     *                           be re planned according to a certain frequency. At this time,
     *                           the global path planning needs to be updated to remove the path that
     *                           the robot passes through.
     * @param tf:
     * @param global_pose The global pose of the robot.
     * @param global_plan the plan to be update and prune.
     * @param dist_behind_robot the distance which should be cut.
     * @return True if the plan changed, otherwise false.
     */
    bool updateAndPrunePlan(const tf2_ros::Buffer* tf, const geometry_msgs::PoseStamped& global_pose,
                            std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);
    /**
     * @brief transformGlobalPlan:transform the plan and pose to global_costmap frame. Since the
     *                            frame of global_plan is unknown, the obstacle obtained by
     *                            costmap is under global_costmap, so the coordinates must be
     *                            unified before path planning.
     * @param tf:A pointer to tf transform
     * @param global_plan:the global plan
     * @param global_pose:the robot pose
     * @param costmap:used costmap
     * @param global_frame:global frame id
     * @param max_plan_length:the max number of plan
     * @param transformed_plan[out]:the transformed result
     * @param current_goal_idx:Since we only convert a part of the plan within the scope of
     *                         the local costmap, we need to record the target points in
     *                         this part of the plan.
     * @param tf_plan_to_global:tf from plan to global
     * @return true if transform success, otherwise false
     */
    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             const geometry_msgs::PoseStamped& global_pose,  const costmap_2d::Costmap2D& costmap,
                             const std::string& global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                             int* current_goal_idx = NULL, geometry_msgs::TransformStamped* tf_plan_to_global = NULL) const;
    /**
     * @brief getFootprintFromParam:we get the footprint throught node param
     * @param nh:the node
     * @return A pointer to FootPrint
     */
    boost::shared_ptr<FootPrint> getFootprintFromParam(const ros::NodeHandle& nh);
    /**
     * @brief estimateLocalGoalOrientation:estimate the local goal's pose.Because when we
     *                                     prune the global plan, the last point became
     *                                     the new goal. But this local goal has no pose.
     *                                     We need to recalculate a smooth pose. Of course,
     *                                     if we don't calculate it, we can use the pose of
     *                                     two adjacent points in the global plan as the new
     *                                     pose. But this may not be smooth.
     * @param global_plan:the global_plan container
     * @param local_goal:the local goal
     * @param current_goal_idx:the index we begin
     * @param tf_plan_to_global:tf from plan to global
     * @param moving_average_length:the number of plan point we use for the compute.
     * @return pose theta.
     */
    double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
                                        int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length=3) const;
    /**
     * @brief costmapConverter2Obstacles:costmapConvert converts points in costmap into different types of
     *                                   obstacles. So we get different types of obstacles. The conversion
     *                                   between the two is just to determine the type and then construct it.
     */
    void costmapConverter2Obstacles();
    /**
     * @brief costmap2Obstacles:convert costmap to point type obstacles.
     *                          That means evey point in costmap can be an obstacle.
     */
    void costmap2Obstacles();
    /**
     * @brief pruneVelocity:The out of bounds parameters are trimmed to meet the requirements.
     *                      There are two ways to cut. One is equal scale clipping, the other is only
     *                      clipping out of bounds parameters. It depends on the user's settings.
     * @param vx:the current velocity x
     * @param vy:the current velocity y
     * @param omega:the current angle velocity
     * @param max_vel_x:the limit of velocity x
     * @param max_vel_y:the limit of velocity y
     * @param max_vel_theta:the limit of angle velocity
     * @param max_vel_x_backwards:the max backward velocity
     * @param is_proportional:whether to prune paramethers by equal scale clipping.
     */
    void pruneVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                          double max_vel_theta, double max_vel_x_backwards,bool is_proportional = false) const;
protected://!<protected variable
    tf2_ros::Buffer* tf_;//!< A pointer to a transform listener
    costmap_2d::Costmap2DROS *costmap_ros_;//!< A pointer to costmap2DROS
    costmap_2d::Costmap2D *costmap_;//!<A pointer to costmap2D
    boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;//!<A pointer to costmap model
    base_local_planner::OdometryHelperRos odom_helper_;//!<A interface to get robot current velocity

    boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter
    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime

    Configuration config_;//!<Point to configuration
    std::vector<boost::shared_ptr<Obstacle> > obstacles_;//!<Container for obstacles
    std::vector<geometry_msgs::PoseStamped> global_plan_container_;//!< A containner for global plan

    bool is_reached_goal_;//!<flag for recording the goal reached status
    Planner planner_;//!<Instance of Planner
    bool is_initialize_;//!< flag for initialize
    Pose2D current_pose_;//!<current pose and position
    geometry_msgs::Twist robot_velocity_;//!current velocity and angle velocity
    Pose2D goal_pose_;//!<goal pose and position
    std::string global_frame_;//!<the frame in which the planner run.
    std::string robot_base_frame_;//!< the base frame of robot
    boost::shared_ptr<FootPrint> robot_footprint_model_;//!<A pointer to robot's footprint. this one come from configuration
    std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot. this one come from costmap
    double footprint_inscribed_radius_;//!< the inscribed radius of footprint
    double footprint_circumscribed_radius_;//!< the circumscribed radius of footprint
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
