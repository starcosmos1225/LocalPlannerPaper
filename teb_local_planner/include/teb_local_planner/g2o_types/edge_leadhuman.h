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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_LEADHUMAN_H_
#define EDGE_LEADHUMAN_H_

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>



namespace teb_local_planner
{

class EdgeLeadHuman : public BaseTebUnaryEdge<1, const geometry_msgs::PoseStamped*, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeLeadHuman()
  {
    _measurement = NULL;
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setPlan() on EdgeObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    double dist = sqrt(pow((bandpt->pose().x() - _measurement->pose.position.x),2)+pow((bandpt->pose().y() - _measurement->pose.position.y),2));
    double error = dist - cfg_->leadHuman.max_dist;
    // Original obstacle cost.
    _error[0] = std::pow(penaltyBoundToInterval(error, cfg_->leadHuman.noise, 0.0),2*cfg_->leadHuman.gamma);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeLeadHuman::computeError() _error[0]=%f\n",_error[0]);
  }
  
  /**
   * @brief Set pointer to associated obstacle for the underlying cost function 
   * @param obstacle 2D position vector containing the position of the obstacle
   */ 
  void setHumanPose(const geometry_msgs::PoseStamped* human_pose)
  {
    _measurement = human_pose;
  }
    
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */ 
  void setParameters(const TebConfig& cfg,  const geometry_msgs::PoseStamped* human_pose)
  {
    cfg_ = &cfg;
    _measurement = human_pose;
  }
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}
#endif
