#include <teb_local_planner/dahu_planner.h>
#include <teb_local_planner/g2o_types/edge_collision.h>
#include <teb_local_planner/g2o_types/edge_manipulation.h>
#include <teb_local_planner/g2o_types/edge_in_area.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>


namespace teb_local_planner {


DahuPlanner::DahuPlanner() {}

DahuPlanner::DahuPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual)
{    
  initialize(cfg, obstacles, robot_model, visual);
}

DahuPlanner::~DahuPlanner()
{
  clearGraph();
  
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void DahuPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual)
{    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);
  
  initialized_ = true;
}


void DahuPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void DahuPlanner::visualize()
{
  ROS_ASSERT(false && "NOT IMPLEMENTED");
}


/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> DahuPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
//   static boost::once_flag flag = BOOST_ONCE_INIT;
//   boost::call_once(&registerG2OTypes, flag);  

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  TEBLinearSolver* linearSolver = new TEBLinearSolver(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  TEBBlockSolver* blockSolver = new TEBBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}

void DahuPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost) {
    ROS_ASSERT(false && "NOT_IMPLEMENTED");
}

void DahuPlanner::clearGraph()
{
  // clear optimizer states
  //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
  
  // TODO: clean up
  optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
  optimizer_->clear();	
}



std::shared_ptr<Plan const> DahuPlanner::plan(std::shared_ptr<dahu_msgs::PlanReq const> planReq) {
  // ROS_INFO("DahuPlanner::plan()");

  _plan = std::make_shared<Plan>(planReq);

  // record all vertices
  for(int i=0; i<_plan->numObjectPoses(); i++) {
    auto vi = _plan->refToObjectPose(i);
    optimizer_->addVertex(vi);
  }
  for(int i=0; i<_plan->numRobotPoses(); i++) {
    auto vi = _plan->refToRobotPose(i);
    optimizer_->addVertex(vi);
  }

  // make sure no two object poses are overlapping
  // todo: missing temporal component
  const double OBJECT_DISTANCE_TOLERANCE = 0.05;
  const double OBJECT_DISTANCE_WEIGHT = 1.0;

  for(int i=0; i<_plan->numObjectPoses(); i++) {
    auto vi = _plan->refToObjectPose(i);
    const dahu_msgs::ObjectPose oi = _plan->getObjectPose(i);
    const double radiusI = _plan->getObject(oi.object_id).radius;
    for(int j=0; j<i; j++) {
      auto vj = _plan->refToObjectPose(j);
      const dahu_msgs::ObjectPose oj = _plan->getObjectPose(j);
      const double radiusJ = _plan->getObject(oj.object_id).radius;
      const double minDist = radiusI + radiusJ + OBJECT_DISTANCE_TOLERANCE;
      EdgeMinDistance* e = new EdgeMinDistance(vi, vj, minDist, OBJECT_DISTANCE_WEIGHT);
      optimizer()->addEdge(e);
    }
  }

  const double IN_AREA_WEIGHT = 1.0;

  for(dahu_msgs::ObjectIn objIn : _plan->request()->constraints.objects_in) {
    auto vertex = _plan->refToObjectPose(objIn.object_pose_id);
    auto objPose = _plan->getObjectPose(objIn.object_pose_id);
    auto object = _plan->getObject(objPose.object_id);
    auto area = _plan->getArea(objIn.area_id);

    const Eigen::Vector2d target(area.center.x, area.center.y);

    const double max_distance = area.radius - object.radius*2.0;
    EdgeInArea* edge = new EdgeInArea(*cfg_, vertex, target, max_distance, IN_AREA_WEIGHT);
    optimizer()->addEdge(edge);
  }

  for(dahu_msgs::CanManipulate c: _plan->request()->constraints.manipulations) {
    auto rv = _plan->refToRobotPose(c.robot_pose_id);
    dahu_msgs::RobotPose rp = _plan->getRobotPose(c.robot_pose_id);
    dahu_msgs::RobotModel rm = _plan->getRobot(rp.robot_id);
    auto ov = _plan->refToObjectPose(c.object_pose_id);
    std::cout << ov <<" -- "<< rv << std::endl;
    std::cout << rm.min_manipulation_distance << " :::: " << rm.max_manipulation_distance << std::endl;
    EdgeManipulation* e = new EdgeManipulation(rv, ov, rm.min_manipulation_distance, rm.max_manipulation_distance, rm.manipulation_cone_angle, 1); // TODO use config weight
    optimizer()->addEdge(e);
  }

  // optimize graph
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int no_iterations = 5;
  optimizer_->optimize(no_iterations);

  clearGraph();
  return _plan;
}

}