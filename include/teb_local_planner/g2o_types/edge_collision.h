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
#ifndef EDGE_COLLISION_H_
#define EDGE_COLLISION_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include "g2o/core/base_unary_edge.h"


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
class EdgeMinDistance : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeMinDistance(VertexPose* v1, VertexPose* v2, double min_distance, double weight) :
    _min_distance(min_distance)
  {
    ROS_ASSERT(v1 != v2);
    setVertex(0, v1);
    setVertex(1, v2);
    Eigen::Matrix<double,1,1> information;
    information.fill(weight);
    setInformation(information);
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    double dist = (conf1->position() - conf2->position()).norm();
    
    _error[0] = penaltyBoundFromBelow( dist , _min_distance, 0);
    ROS_ASSERT(std::isfinite(_error[0]));
  }

protected:
  const double _min_distance;  
    
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
    

} // end namespace

#endif
