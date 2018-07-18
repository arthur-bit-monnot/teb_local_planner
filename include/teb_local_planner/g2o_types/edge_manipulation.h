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
#ifndef EDGE_MANIPULATION_H_
#define EDGE_MANIPULATION_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include "g2o/core/base_unary_edge.h"
#include <cmath>


namespace teb_local_planner
{

/**
 * @class EdgePreferRotDir
 * @brief Edge defining the cost function for penalzing a specified turning direction, in particular left resp. right turns
 * 
 * The edge depends on two consecutive vertices \f$ \mathbf{s}_i, \mathbf{s}_{i+1} \f$ and penalizes a given rotation direction
 * based on the \e weight and \e dir (\f$ dir \in \{-1,1\} \f$)
 * \e dir should be +1 to prefer left rotations and -1 to prefer right rotations  \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgePreferRotDir
 */     
class EdgeManipulation : public BaseTebBinaryEdge<2, double, VertexPose, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeManipulation(VertexRobotPose* robot_pose, VertexObjectPose* object_pose, double min_distance, double max_distance, double cone_size, double weight) :
    _min_distance(min_distance), _max_distance(max_distance), _cone_size(cone_size)
  {
    ROS_ASSERT(robot_pose != object_pose);
    setVertex(0, robot_pose);
    setVertex(1, object_pose);
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = weight;
    information(1,1) = weight;
    setInformation(information);
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    const VertexRobotPose* robot_pose = static_cast<const VertexRobotPose*>(_vertices[0]);
    const VertexObjectPose* object_pose = static_cast<const VertexObjectPose*>(_vertices[1]);
    auto vec = object_pose->position() - robot_pose->position();
    double dist = vec.norm();
    
    
    _error[0] = penaltyBoundToInterval( dist , _min_distance, _max_distance, 0);
    
    const double target_angle = atan2(vec[1], vec[0]);
    const double angular_distance = angle_difference(target_angle, robot_pose->theta());
    _error[1] = penaltyBoundFromAbove( angular_distance, _cone_size/2.0, 0);
    ROS_ASSERT(std::isfinite(_error[0]));
    ROS_ASSERT(std::isfinite(_error[1]));
  }

protected:
  const double _min_distance;
  const double _max_distance;  
  const double _cone_size; // size of the cone (radians) in which manipulation must be done
    
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
    

} // end namespace

#endif
