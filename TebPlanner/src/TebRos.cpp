#include <TebPlanner/TebRos.h>

namespace TebPlanner
{
TebRos::TebRos():
    tf_(NULL),
    costmap_ros_(NULL),
    is_reached_goal_(false),
    robot_footprint_model_(NULL),
    is_initialize_(false)
{}
TebRos::~TebRos(){}
void TebRos::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    if (!is_initialize_)
    {
        ros::NodeHandle nh("~/" + name);
        config_.loadRosParamFromNodeHandle(nh);
        costmap_ros_ = costmap_ros;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        costmap_2d::calculateMinAndMaxDistances(footprint_spec_,footprint_inscribed_radius_,footprint_circumscribed_radius_);
        global_frame_ = costmap_ros->getGlobalFrameID();
        robot_base_frame_ = costmap_ros->getBaseFrameID();
        robot_footprint_model_ = getFootprintFromParam(nh);
        planner_.initialize(&config_,&obstacles_,robot_footprint_model_);
        if (!config_.obstacles.costmap_converter_plugin.empty())
        {
          try
          {
            costmap_converter_ = costmap_converter_loader_.createInstance(config_.obstacles.costmap_converter_plugin);
            std::string converter_name = costmap_converter_loader_.getName(config_.obstacles.costmap_converter_plugin);
            // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
            boost::replace_all(converter_name, "::", "/");
            costmap_converter_->setOdomTopic(config_.odom_topic);
            costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
            costmap_converter_->setCostmap2D(costmap_);

            costmap_converter_->startWorker(ros::Rate(config_.obstacles.costmap_converter_rate), costmap_, config_.obstacles.costmap_converter_spin_thread);
            ROS_INFO_STREAM("Costmap conversion plugin " << config_.obstacles.costmap_converter_plugin << " loaded.");
          }
          catch(pluginlib::PluginlibException& ex)
          {
            ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
            costmap_converter_.reset();
          }
        }
        else
          ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
        is_initialize_ = true;
        ROS_INFO("TebRos initialized");

    }else
    {
        ROS_ERROR("Error: Re-initialize The TebRos");
    }
}
bool TebRos::setPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan)
{
    if (!is_initialize_)
    {
        ROS_ERROR("Error:No Initialize before setPlan");
        return false;
    }
    global_plan_container_.clear();
    global_plan_container_ = global_plan;
    is_reached_goal_ = false;
    return true;
}
bool TebRos::computeVelocityCommands(geometry_msgs::TwistStamped &cmd_vel)
{
    // check init
    if (!is_initialize_)
    {
        ROS_ERROR("Error:No Initialize before setPlan");
        return false;
    }
    //set default velocity and angle velocity
    static uint32_t seq = 0;
    cmd_vel.header.seq = seq++;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.header.frame_id = robot_base_frame_;
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    // get the robot pose
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    current_pose_ = Pose2D(robot_pose);
    // get the robot velocity
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    robot_velocity_.linear.x = robot_vel.pose.position.x;
    robot_velocity_.linear.y = robot_vel.pose.position.y;
    robot_velocity_.angular.z = tf2::getYaw(robot_vel.pose.orientation);
    // prune the plan
    updateAndPrunePlan(tf_,robot_pose,global_plan_container_,config_.trajectory.global_plan_prune_distance);
    // transform the plan to global frame
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    geometry_msgs::TransformStamped plan_to_global;
    int current_goal_index;
    if (!transformGlobalPlan(*tf_,global_plan_container_,robot_pose,costmap_,global_frame_,
                             config_.trajectory.max_global_plan_lookahead_dist,
                             transformed_plan,&current_goal_index,plan_to_global))
    {
        ROS_ERROR("transform global plan fail!");
        return false;
    }
    // check whether reached the goal
    geometry_msgs::PoseStamped goal_pose_transformed;
    tf2::doTransform(global_plan_container_.back(),goal_pose_transformed,plan_to_global);
    Pose2D local_goal(goal_pose_transformed);
    double distance = (local_goal.getPosition()-current_pose_.getPosition()).norm();
    double delta_theata = g2o::normalize_theta(local_goal.getPose()-current_pose_.getPose());
    if (distance<config_.goal_tolerance.xy_goal_tolerance&&fabs(delta_theata)<config_.goal_tolerance.yaw_goal_tolerance)
    {
        ROS_INFO("TebPlanner:reach goal");
        is_reached_goal_ = true;
        return true;
    }
    if (transformed_plan.empty())
    {
        ROS_ERROR("TebPlanner:transformed_plan is empty");
        return false;
    }
    // Estimate the pose of a local goal
    if (config_.trajectory.global_plan_overwrite_orientation)
    {
        double theta = estimateLocalGoalOrientation(global_plan_container_,
                                                    goal_pose_transformed,
                                                    current_goal_index,
                                                    plan_to_global);
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        tf2::convert(q, transformed_plan.back().pose.orientation);
    }
    if (transformed_plan.size()==1)
    {
        transformed_plan.insert(transformed_plan.begin(),geometry_msgs::PoseStamped());
    }
    transformed_plan[0] = robot_pose;
    // get obstacles
    obstacles_.clear();
    if (costmap_converter_)
    {
        costmapConverter2Obstacles();
    }else
        costmap2Obstacles();
    // plan the local point
    if (!planner_.plan(transformed_plan,&robot_vel))
    {
        planner_.clearPlanner();
        ROS_ERROR("TebPlanner:fail to plan");
        return false;
    }
    // check whether the plan feasible
    if (!planner_.checkFeasible(costmap_model_.get(),footprint_spec_,
                                footprint_inscribed_radius_, footprint_circumscribed_radius_,
                                config_.trajectory.feasibility_check_no_poses))
    {
        planner_.clearPlanner();
        ROS_ERROR("TebPlanner:the local plan can not feasible!");
        return false;
    }
    // ask the current velocity and angle velocity
    if (!planner_.getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, config_.trajectory.control_look_ahead_poses))
    {
        planner_.clearPlanner();
        ROS_ERROR("TebPlanner:can not get velocity command from planner");
        return false;
    }
    pruneVelocity(cmd_vel.twist.linear.x,cmd_vel.twist.linear.y,cmd_vel.twist.angular.z,
                  config_.robot.max_vel_x,config_.robot.max_vel_y,config_.robot.max_vel_theta,
                  config_.robot.max_vel_x_backwards,config_.robot.use_proportional_saturation);
    // publish plan?it can skip
    // publisPlan();
    return true;

}

bool TebRos::isGoalReached()
{
    return is_reached_goal_;
}

bool TebRos::cancel()
{
    return false;
}

bool TebRos::updateAndPrunePlan(const tf2_ros::Buffer *tf,
                                const geometry_msgs::PoseStamped &global_pose,
                                std::vector<geometry_msgs::PoseStamped> &global_plan,
                                double dist_behind_robot)
{
    // check whether the global plan empty
    if (global_plan.empty())
    {
        return false;
    }
    //transform the global_pose to plan
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot_pose;
    tf2::doTransform(global_pose, robot_pose, global_to_plan_transform);
    //search a nearest plan point and cut off any point before it.
    int cutoff_index = -1;
    for(int i=0;i<global_plan.size();++i)
    {
        double x = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double distance = x*x + y*y;
        if (distance<dist_behind_robot)
        {
            cutoff_index = i;
            break;
        }
    }
    if (cutoff_index==-1)
        return false;
    if (cutoff_index!=0)
    {
        global_plan.erase(global_plan.begin(),global_plan.begin()+cutoff_index);
        return true;
    }
    return false;
}




}
}
