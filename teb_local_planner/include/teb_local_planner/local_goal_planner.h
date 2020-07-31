/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef TEB_LOCAL_GOAL_PLANNER_H_1
#define TEB_LOCAL_GOAL_PLANNER_H_1

#include <math.h>


// teb stuff
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>

// g2o lib stuff
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>


#include <nav_msgs/Odometry.h>
#include <limits.h>

// boost
#include <boost/thread.hpp>

namespace teb_local_planner
{
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  TEBBlockSolver;

//! Typedef for the linear solver utilized for optimization
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;
class TebLocalGoalPlanner
{
public:
    TebLocalGoalPlanner();
    TebLocalGoalPlanner(const TebConfig* cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>());
    ~TebLocalGoalPlanner();
    void initialize(const TebConfig* cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>());

  bool plan(std::vector<geometry_msgs::PoseStamped>* global_plan, const geometry_msgs::PoseStamped& human_pose);
  PoseSE2 getOptimalGoal();
  bool optimizeTEB(int iterations_innerloop, bool compute_cost_afterwards = false,
                   double obst_cost_scale=1.0);
  void setObstVector(ObstContainer* obst_vector) {obstacles_ = obst_vector;}
  ObstContainer& getObstVector() const {return *obstacles_;}
  static void registerG2OTypes();
  
  boost::shared_ptr<g2o::SparseOptimizer> optimizer() {return optimizer_;};
  boost::shared_ptr<const g2o::SparseOptimizer> optimizer() const {return optimizer_;};
  bool isOptimized() const {return optimized_;};
  void computeCurrentCost(double obst_cost_scale=1.0);
  void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0)
  {
    computeCurrentCost(obst_cost_scale);
    cost.push_back( getCurrentCost() );
  }
  double getCurrentCost() const {return cost_;}
  
protected:
  bool buildGraph(double weight_multiplier=1.0);
  bool optimizeGraph(int no_iterations, bool clear_after=true);
  void clearGraph();
  void computeInitPose(geometry_msgs::PoseStamped& constrain_pose,const std::vector<geometry_msgs::PoseStamped>* global_plan);
  void AddVertices();
  void AddEdgesObstacle(double weight_multiplier=1.0);
  void AddEdgesHumanPose();
  void AddEdgesPlan();
  boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();

  std::vector<geometry_msgs::PoseStamped>* global_plan_;
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
  ObstContainer obstacles_with_vertex;
  double cost_; //!< Store cost value of the current hyper-graph
  RobotFootprintModelPtr robot_model_; //!< Robot model
  boost::shared_ptr<g2o::SparseOptimizer> optimizer_; //!< g2o optimizer for trajectory optimization
  bool initialized_; //!< Keeps track about the correct initialization of this class
  bool optimized_; //!< This variable is \c true as long as the last optimization has been completed successful
  geometry_msgs::PoseStamped human_pose_;//!< the human's position and pose
  VertexPose* local_goal_;//!< the local goal which we need to optimize
  //Eigen::Vector2d min_distance_line_[2];//!< the minimum distance line. used for constrain the local point.
  int min_point_index_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
};

//! Abbrev. for shared instances of the TebOptimalPlanner
typedef boost::shared_ptr<TebLocalGoalPlanner> TebLocalGoalPlannerPtr;
//! Abbrev. for shared const TebOptimalPlanner pointers
typedef boost::shared_ptr<const TebLocalGoalPlanner> TebLocalGoalPlannerConstPtr;
//! Abbrev. for containers storing multiple teb optimal planners
typedef std::vector< TebLocalGoalPlannerPtr > TebLocalGoalPlannerContainer;

} // namespace teb_local_planner

#endif /* OPTIMAL_PLANNER_H_ */
