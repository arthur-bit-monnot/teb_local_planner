#include <teb_local_planner/dahu_planner.h>
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
  ROS_ASSERT(false && "NOT IMPLEMENTED")
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
    ROS_ASSERT(false && "NOT_IMPLEMENTED")
}

void DahuPlanner::clearGraph()
{
  // clear optimizer states
  //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
  optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
  optimizer_->clear();	
}



void DahuPlanner::plan(const dahu::Plan& plan) {

  // build graph
  Eigen::Vector2d* target = new Eigen::Vector2d(10.0, 2.0);
  vertex = new VertexPose();
  vertex->setId(0);
  optimizer_->addVertex(vertex);

  EdgeInArea* edge = new EdgeInArea(*cfg_, vertex, target, 1.0);
  optimizer_->addEdge(edge);



  // optimize graph
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int no_iterations = 10;
  int iter = optimizer_->optimize(no_iterations);

  std::cout << "V: " << vertex->x() << " : " << vertex->y() << std::endl;

  //clearGraph();
}

}