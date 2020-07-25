#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Core>

#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <TebPlanner/Obstacle.h>
#include <TebPlanner/Configuration.h>

namespace TebPlanner
{
/**
 * @brief The Planner class:the core of TebPlanner.
 *                          It accepts ros data and divides the plan into TEB local points and time interval
 *                          sets. And construct the optimizer according to the optimization constraints to
 *                          optimize these local points and time. It can give the speed and angular velocity
 *                          that ros needs to execute according to the current state so that move_base can
 *                          control the robot's movement.
 */
class Planner
{
public:
    /**
     * @brief Planner:the default constructor
     */
    Planner();
    /**
     * @brief ~Planner:the default destructor
     */
    ~Planner();
    /**
     * @brief initialize:init the pointer of configuration,obstacles,footprint and leaded_point
     * @param config:A pointer to Teb Configuration
     * @param obstacles:A pointer to containner of obstacles
     * @param robot_model:A pointer to FootPrint
     * @param leaded_point:A pointer to leaded point
     */
    void initialize(const TebConfig* config,
                    std::vector<boost::shared_ptr<Obstacle>>* obstacles=NULL,
                    boost::shared_ptr<FootPrint> robot_model = NULL,
                    Eigen::Vector2d* leaded_point = NULL);
    /**
     * @brief registerG2oType:regist ourselves' g2o edge and vertex type
     */
    void registerG2oType();
    /**
     * @brief plan:the core function for planning a path
     * @param initial_plan:the global planning given by move_base
     * @param start_vel:the start velocity
     * @param free_goal_vel:true if reach goal with free velocity, otherwise false
     * @return true if plan path success, otherwise false
     */
    bool plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan,
              const geometry_msgs::Twist* start_vel = NULL,
              bool free_goal_vel=false);
    /**
     * @brief plan:the core function for planning a path
     * @param start:the start pose and angle
     * @param goal:the goal pose and angle
     * @param start_vel:the start velocity
     * @param free_goal_vel:true if reach goal with free velocity, otherwise false
     * @return true if plan path success, otherwise false
     */
    bool plan(const tf::Pose& start,
              const tf::Pose& goal,
              const geometry_msgs::Twist* start_vel = NULL,
              bool free_goal_vel=false);
    /**
     * @brief plan:the core function for planning a path
     * @param start:the start pose define by Pose2D
     * @param goal:the goal pose define by Pose2D
     * @param start_vel:the start velocity
     * @param free_goal_vel:true if reach goal with free velocity, otherwise false
     * @return true if plan path success, otherwise false
     */
    bool plan(const Pose2D& start,
              const Pose2D& goal,
              const geometry_msgs::Twist* start_vel = NULL,
              bool free_goal_vel=false);
    /**
     * @brief initOptimizer:create a g2o optimizer
     * @return A pointer to optimizer
     */
    boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();
    /**
     * @brief buildGraph:build g2o graph by add vertex and edge
     * @param weight_multiplier: the optimizer weight scale
     * @return: true if build success, otherwise false
     */
    bool buildGraph(double weight_multiplier=1.0);
    /**
     * @brief clearGraph:clear the graph
     */
    void clearGraph();
    /**
     * @brief clearPlanner:clear the graph and Teb band
     */
    virtual void clearPlanner();
    /**
     * @brief getVelocityCommand:get the velocity in current status
     * @param vx:the x-linear velocity
     * @param vy:the y-linear velocity
     * @param theta:the angle velocity
     * @param number_poses:the number of poses used to compute the velocity
     * @return true if compute success, otherwise false
     */

    bool getVelocityCommand(double& vx, double& vy, double& theta, int number_poses);
    /**
     * @brief optimizeTEB:optimize the built graph
     * @param iterations_innerloop:the g2o optimizer's loop times.
     * @param iterations_outerloop:the graph's loop times out of the g2o optimizer.
     * @param compute_cost_afterwards:whether compute the cost after optimize
     * @param obst_cost_scale:the obstacle's cost scale.we can modify it to set how important the obstacles are
     *                        in the optimizer.
     * @param leaded_point_cost_scale: the leaded point's cost scale. we can modify it to set how important the
     *                                 leaded point are in the optimizer.
     * @param alternative_time_cost
     * @return true if optimize success, otherwise false
     */
    bool optimizeTEB(int iterations_innerloop,
                     int iterations_outerloop,
                     bool compute_cost_afterwards = false,
                     double obst_cost_scale=1.0, double leaded_point_cost_scale=1.0,
                     bool alternative_time_cost=false);
    /**
     * @brief checkFeasible:check whether the planner's local point can through the obstacles
     * @param costmap_model:use costmap to check feasible
     * @param footprint_spec:the footprint in current world
     * @param inscribed_radius:the footprint's inscribed radius.
     * @param circumscribed_radius:the footprint's circumscribed_radius
     * @param look_ahead_idx:the number of look ahead point(since we need not check every point in the plan)
     * @return true if feasible, otherwise false
     */
    bool checkFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                                 double inscribed_radius, double circumscribed_radius, int look_ahead_idx);
    /**
     * @brief setObstacles:set the obstacles's pointer
     * @param obstacles:pointer to contaner of obstacles
     */
    void setObstacles(const std::vector<Obstacle*>* obstacles);
    /**
     * @brief getObstacles:get the pointer of obstacles
     * @return A pointer to container of obstacles
     */
    std::vector<Obstacles*>* getObstacles();
    /**
     * @brief setLeadedPoint:set leaded point's location
     * @param leaded_point:leaded point's location
     */
    void setLeadedPoint(Eigen::Vector2d* leaded_point);
    /**
     * @brief getLeadedPoint:get leaded point's location
     * @return leaded point
     */
    Eigen::Vector2d* getLeadedPoint();
    /**
     * @brief setFootPrint:set pointer of robot_model
     * @param robot_model: A pointer to footprint
     */
    void setFootPrint(FootPrint* robot_model);
    /**
     * @brief getFootPrint:get the footprint
     * @return A pointer to footprint
     */
    FootPrint* getFootPrint();
    // Add Edges and Vertexs
    /**
     * @brief AddObstacleEdge:Add obstacles constrain to g2o edge
     */
    void AddObstacleEdge();
    /**
     * @brief AddVelocityEdge:Add velocity constrain to g2o edge
     */
    void AddVelocityEdge();
    /**
     * @brief AddLeadedPointEdge:Add leaded point constrain to g2o edge
     */
    void AddLeadedPointEdge();
    /**
     * @brief AddAccelerationEdge:Add acceleration constrain to g2o edge
     */
    void AddAccelerationEdge();
    /**
     * @brief AddPreferRotateEdge:Add prefer rotate constrain to g2o edge
     */
    void AddPreferRotateEdge();
    /**
     * @brief AddShortestPathEdge:Add shortest path constrain to g2o edge
     */
    void AddShortestPathEdge();
    /**
     * @brief AddTimeDiffEdge:Add fastest time constrain to g2o edge
     */
    void AddTimeDiffEdge();
    /**
     * @brief AddKinematicEdge:Add kinematic constrain to g2o edge
     */
    void AddKinematicEdge();
    /**
     * @brief AddVelocityNearObstacleEdge:Add velocity near obstacles constrain to g2o edge
     */
    void AddVelocityNearObstacleEdge();
    /**
     * @brief AddPoseVertex:Add local point to Vertex
     */
    void AddPoseVertex();
    /**
     * @brief AddTimeDiffVertex:Add time diff to Vertex
     */
    void AddTimeDiffVertex();
protected:
    Teb teb_;//!<the teb data struct for local point and time diff
    const Configuration* config_;//!<A pointer of configuration
    std::vector<boost::shared_ptr<Obstacle>>* obstacles_;//!<container of Obstalces
    boost::shared_ptr<FootPrint> robot_model_;//!<A pointer of footprint
    boost::shared_ptr<g2o::SparseOptimizer> optimizer_;//!<A pointer of g2o optimizer.
    bool initialized_; //!< initialized flag
    bool optimized_; //!< true if optimize success, otherwise false.
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
