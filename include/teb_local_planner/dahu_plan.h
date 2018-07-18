#ifndef _TEB_DAHU_PLAN_H_
#define _TEB_DAHU_PLAN_H_

#include <dahu_msgs/PlanReq.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>

namespace teb_local_planner {


class Plan {
public:
  std::shared_ptr<dahu_msgs::PlanReq const> _request;

  std::vector<VertexObjectPose *> _object_poses;
  std::vector<VertexRobotPose *> _robot_poses;

  dahu_msgs::RobotModel getRobot(std::string robot_id) const {
    for(dahu_msgs::RobotModel a : _request->robots) {
      if(a.model_id == robot_id)
        return a;
    }
    ROS_ERROR("Robot name not found %s", robot_id.c_str()); 
    throw std::exception();
  }

  dahu_msgs::Area getArea(std::string name) const {
    for(dahu_msgs::Area a : _request->areas) {
      if(a.name == name)
        return a;
    }
    ROS_ERROR("Area name not found %s", name.c_str()); 
    throw std::exception();
  }

  dahu_msgs::Object getObject(std::string name) const {
    for(dahu_msgs::Object a : _request->objects) {
      if(a.name == name)
        return a;
    }
    ROS_ERROR("Object name not found %s", name.c_str()); 
    throw std::exception();
  }

  dahu_msgs::ObjectPose getObjectPose(int poseId) const {
    ROS_ASSERT(poseId < _request->object_poses.size());
    return _request->object_poses[poseId];
  }

  dahu_msgs::RobotPose getRobotPose(int poseId) const {
    ROS_ASSERT(poseId < _request->robot_poses.size());
    return _request->robot_poses[poseId];
  }

  std::shared_ptr<dahu_msgs::PlanReq const> request() const { return _request; }


public:
  Plan(std::shared_ptr<dahu_msgs::PlanReq const> req) : _request(req) {
    for(uint i=0 ; i < req->object_poses.size(); i++) {
      VertexPose* v = new VertexPose(i, i, i, false);
      v->setId(i);
      _object_poses.push_back(v);
    }
    for(uint i=0 ; i < req->robot_poses.size(); i++) {
      VertexPose* v = new VertexPose(i +1, i, i, false);
      v->setId(i + _object_poses.size());
      _robot_poses.push_back(v);
    }
    
  }

  int numObjectPoses() const { return _object_poses.size(); }
  int numRobotPoses() const { return _robot_poses.size(); }

  VertexObjectPose* refToObjectPose(int pose_id) const {
    ROS_ASSERT(pose_id < _object_poses.size());
    ROS_ASSERT(_object_poses[pose_id] != NULL);
    return _object_poses[pose_id]; 
  }
  VertexRobotPose* refToRobotPose(int pose_id) const {
    ROS_ASSERT(pose_id < _robot_poses.size());
    ROS_ASSERT(_robot_poses[pose_id] != NULL);
    return _robot_poses[pose_id]; 
  }

  ~Plan() {
    for(auto v : _object_poses)
      delete v;
    for(auto v : _robot_poses)
      delete v;
  }
};

}

#endif