#ifndef _DAHU_PLANNER_H_
#define _DAHU_PLANNER_H_

#include <math.h>


// teb stuff
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/robot_footprint_model.h>

// g2o lib stuff
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <teb_local_planner/TrajectoryMsg.h>

#include <nav_msgs/Odometry.h>
#include <limits.h>

#include <dahu/Plan.h>

namespace teb_local_planner {

class DahuPlanner
{
public:
    
  /**
   * @brief Default constructor
   */
  DahuPlanner();
  
  /**
   * @brief Construct and initialize the TEB optimal planner.
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
   * @param visual Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  DahuPlanner(const TebConfig& cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                    TebVisualizationPtr visual = TebVisualizationPtr());
  
  /**
   * @brief Destruct the optimal planner.
   */
  virtual ~DahuPlanner();
  
  /**
    * @brief Initializes the optimal planner
    * @param cfg Const reference to the TebConfig class for internal parameters
    * @param obstacles Container storing all relevant obstacles (see Obstacle)
    * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
    * @param visual Shared pointer to the TebVisualization class (optional)
    * @param via_points Container storing via-points (optional)
    */
  void initialize(const TebConfig& cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                  TebVisualizationPtr visual = TebVisualizationPtr());
  
  

  /** @name Plan a trajectory  */
  //@{
  
  void plan(const dahu::Plan& plan);

  /**
   * @brief Optimize a previously initialized trajectory (actual TEB optimization loop).
   * 
   * optimizeTEB implements the main optimization loop. \n
   * It consist of two nested loops:
   * 	- The outer loop resizes the trajectory according to the temporal resolution by invoking TimedElasticBand::autoResize().
   * 	  Afterwards the internal method optimizeGraph() is called that constitutes the innerloop.
   * 	- The inner loop calls the solver (g2o framework, resp. sparse Levenberg-Marquardt) and iterates a specified
   * 	  number of optimization calls (\c iterations_innerloop).
   * 
   * The outer loop is repeated \c iterations_outerloop times. \n
   * The ratio of inner and outer loop iterations significantly defines the contraction behavior 
   * and convergence rate of the trajectory optimization. Based on our experiences, 2-6 innerloop iterations are sufficient. \n
   * The number of outer loop iterations should be determined by considering the maximum CPU time required to match the control rate. \n
   * Optionally, the cost vector can be calculated by specifying \c compute_cost_afterwards, see computeCurrentCost().
   * @remarks This method is usually called from a plan() method
   * @param iterations_innerloop Number of iterations for the actual solver loop
   * @param iterations_outerloop Specifies how often the trajectory should be resized followed by the inner solver loop.
   * @param compute_cost_afterwards if \c true Calculate the cost vector according to computeCurrentCost(),
   *         the vector can be accessed afterwards using getCurrentCost().
   * @param obst_cost_scale Specify extra scaling for obstacle costs (only used if \c compute_cost_afterwards is true)
   * @param viapoint_cost_scale Specify extra scaling for via-point costs (only used if \c compute_cost_afterwards is true)
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time 
   *          (only used if \c compute_cost_afterwards is true).
   * @return \c true if the optimization terminates successfully, \c false otherwise
   */	  
  bool optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = false,
                   double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);
  
  //@}
  
  
  /** @name Take obstacles into account */
  //@{
  
  
  /**
   * @brief Assign a new set of obstacles
   * @param obst_vector pointer to an obstacle container (can also be a nullptr)
   * @remarks This method overrids the obstacle container optinally assigned in the constructor.
   */
  void setObstVector(ObstContainer* obst_vector) {obstacles_ = obst_vector;}
  
  /**
   * @brief Access the internal obstacle container.
   * @return Const reference to the obstacle container
   */
  const ObstContainer& getObstVector() const {return *obstacles_;}

  //@}
	  
  
  /** @name Visualization */
  //@{
  
  /**
   * @brief Register a TebVisualization class to enable visiualization routines (e.g. publish the local plan and pose sequence)
   * @param visualization shared pointer to a TebVisualization instance
   * @see visualize
   */
  void setVisualization(TebVisualizationPtr visualization);
  
  /**
   * @brief Publish the local plan and pose sequence via ros topics (e.g. subscribe with rviz).
   * 
   * Make sure to register a TebVisualization instance before using setVisualization() or an overlaoded constructor.
   * @see setVisualization
   */
  virtual void visualize();
  
  //@}
  
  
  /** @name Utility methods and more */
  //@{
        
  /**
   * @brief Reset the planner by clearing the internal graph and trajectory.
   */
  virtual void clearPlanner() 
  {
    clearGraph();
    //teb_.clearTimedElasticBand();
  }
  
  /**
   * @brief Prefer a desired initial turning direction (by penalizing the opposing one)
   * 
   * A desired (initial) turning direction might be specified in case the planned trajectory oscillates between two 
   * solutions (in the same equivalence class!) with similar cost. Check the parameters in order to adjust the weight of the penalty.
   * Initial means that the penalty is applied only to the first few poses of the trajectory.
   * @param dir This parameter might be RotType::left (prefer left), RotType::right (prefer right) or RotType::none (prefer none)
   */
  virtual void setPreferredTurningDir(RotType dir) {prefer_rotdir_=dir;}
  
  /**
   * @brief Register the vertices and edges defined for the TEB to the g2o::Factory.
   * 
   * This allows the user to export the internal graph to a text file for instance.
   * Access the optimizer() for more details.
   */
  static void registerG2OTypes();

  /**
   * @brief Access the internal g2o optimizer.
   * @warning In general, the underlying optimizer must not be modified directly. Use with care...
   * @return const shared pointer to the g2o sparse optimizer
   */
  boost::shared_ptr<g2o::SparseOptimizer> optimizer() {return optimizer_;};
  
  /**
   * @brief Access the internal g2o optimizer (read-only).
   * @return const shared pointer to the g2o sparse optimizer
   */
  boost::shared_ptr<const g2o::SparseOptimizer> optimizer() const {return optimizer_;};
  
  /**
   * @brief Check if last optimization was successful
   * @return \c true if the last optimization returned without errors, 
   *         otherwise \c false (also if no optimization has been called before).
   */
  bool isOptimized() const {return optimized_;};
	
  /**
   * @brief Compute the cost vector of a given optimization problen (hyper-graph must exist).
   * 
   * Use this method to obtain information about the current edge errors / costs (local cost functions). \n
   * The vector of cost values is composed according to the different edge types (time_optimal, obstacles, ...). \n
   * Refer to the method declaration for the detailed composition. \n
   * The cost for the edges that minimize time differences (EdgeTimeOptimal) corresponds to the sum of all single
   * squared time differneces: \f$ \sum_i \Delta T_i^2 \f$. Sometimes, the user may want to get a value that is proportional
   * or identical to the actual trajectory transition time \f$ \sum_i \Delta T_i \f$. \n
   * Set \c alternative_time_cost to true in order to get the cost calculated using the latter equation, but check the 
   * implemented definition, if the value is scaled to match the magnitude of other cost values.
   * 
   * @todo Remove the scaling term for the alternative time cost.
   * @todo Can we use the last error (chi2) calculated from g2o instead of calculating it by ourself?
   * @see getCurrentCost
   * @see optimizeTEB
   * @param obst_cost_scale Specify extra scaling for obstacle costs.
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time.
   * @return TebCostVec containing the cost values
   */
  void computeCurrentCost(double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);
  
  /**
   * Compute and return the cost of the current optimization graph (supports multiple trajectories)
   * @param[out] cost current cost value for each trajectory
   *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
   * @param obst_cost_scale Specify extra scaling for obstacle costs
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
   */
  virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false)
  {
    computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
    cost.push_back( getCurrentCost() );
  }
  
  /**
   * @brief Access the cost vector.
   *
   * The accumulated cost value previously calculated using computeCurrentCost 
   * or by calling optimizeTEB with enabled cost flag.
   * @return const reference to the TebCostVec.
   */
  double getCurrentCost() const {return cost_;}
  
    

  //@}
  
protected:
  
  /** @name Hyper-Graph creation and optimization */
  //@{
  
  /**
   * @brief Build the hyper-graph representing the TEB optimization problem.
   * 
   * This method creates the optimization problem according to the hyper-graph formulation. \n
   * For more details refer to the literature cited in the TebOptimalPlanner class description.
   * @see optimizeGraph
   * @see clearGraph
   * @param weight_multiplier Specify a weight multipler for selected weights in optimizeGraph
   *                          This might be used for weight adapation strategies.
   *                          Currently, only obstacle collision weights are considered.
   * @return \c true, if the graph was created successfully, \c false otherwise.
   */
  bool buildGraph(double weight_multiplier=1.0);
  
  /**
   * @brief Optimize the previously constructed hyper-graph to deform / optimize the TEB.
   * 
   * This method invokes the g2o framework to solve the optimization problem considering dedicated sparsity patterns. \n
   * The current implementation calls a non-constrained sparse Levenberg-Marquardt algorithm. Constraints are considered
   * by utilizing penalty approximations. Refer to the literature cited in the TebOptimalPlanner class description.
   * @see buildGraph
   * @see clearGraph
   * @param no_iterations Number of solver iterations
   * @param clear_after Clear the graph after optimization.
   * @return \c true, if optimization terminates successfully, \c false otherwise.
   */
  bool optimizeGraph(int no_iterations, bool clear_after=true);
  
  /**
   * @brief Clear an existing internal hyper-graph.
   * @see buildGraph
   * @see optimizeGraph
   */
  void clearGraph();
  
  /**
   * @brief Add all relevant vertices to the hyper-graph as optimizable variables.
   * 
   * Vertices (if unfixed) represent the variables that will be optimized. \n
   * In case of the Timed-Elastic-Band poses and time differences form the vertices of the hyper-graph. \n
   * The order of insertion of vertices (to the graph) is important for efficiency,
   * since it affect the sparsity pattern of the underlying hessian computed for optimization.
   * @see VertexPose
   * @see VertexTimeDiff
   * @see buildGraph
   * @see optimizeGraph
   */
//   void AddTEBVertices();
  
  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular velocity.
   * @see EdgeVelocity
   * @see buildGraph
   * @see optimizeGraph
   */
//   void AddEdgesVelocity();
  
  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular acceleration.
   * @see EdgeAcceleration
   * @see EdgeAccelerationStart
   * @see EdgeAccelerationGoal
   * @see buildGraph
   * @see optimizeGraph
   */
//   void AddEdgesAcceleration();
  
  /**
   * @brief Add all edges (local cost functions) for minimizing the transition time (resp. minimize time differences)
   * @see EdgeTimeOptimal
   * @see buildGraph
   * @see optimizeGraph
   */
//   void AddEdgesTimeOptimal();
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from static obstacles
   * @warning do not combine with AddEdgesInflatedObstacles
   * @see EdgeObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
//   void AddEdgesObstacles(double weight_multiplier=1.0);
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from static obstacles (legacy association strategy)
   * @warning do not combine with AddEdgesInflatedObstacles
   * @see EdgeObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
//   void AddEdgesObstaclesLegacy(double weight_multiplier=1.0);
  
  /**
   * @brief Add all edges (local cost functions) related to minimizing the distance to via-points
   * @see EdgeViaPoint
   * @see buildGraph
   * @see optimizeGraph
   */
//   void AddEdgesViaPoints();
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from dynamic (moving) obstacles.
   * @warning experimental 
   * @todo Should we also add neighbors to decrease jiggling/oscillations
   * @see EdgeDynamicObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)

   */
//   void AddEdgesDynamicObstacles(double weight_multiplier=1.0);

  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a differential drive robot
   * @warning do not combine with AddEdgesKinematicsCarlike()
   * @see AddEdgesKinematicsCarlike
   * @see buildGraph
   * @see optimizeGraph
   */
//   void AddEdgesKinematicsDiffDrive();
  
  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a carlike robot
   * @warning do not combine with AddEdgesKinematicsDiffDrive()
   * @see AddEdgesKinematicsDiffDrive
   * @see buildGraph
   * @see optimizeGraph
   */
//   void AddEdgesKinematicsCarlike();
  
  /**
   * @brief Add all edges (local cost functions) for prefering a specifiy turning direction (by penalizing the other one)
   * @see buildGraph
   * @see optimizeGraph
   */
  //void AddEdgesPreferRotDir(); 
  
  //@}
  
  
  /**
   * @brief Initialize and configure the g2o sparse optimizer.
   * @return shared pointer to the g2o::SparseOptimizer instance
   */
  boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();
    

  // external objects (store weak pointers)
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
  
  double cost_; //!< Store cost value of the current hyper-graph
  RotType prefer_rotdir_; //!< Store whether to prefer a specific initial rotation in optimization (might be activated in case the robot oscillates)
  
  // internal objects (memory management owned)
  TebVisualizationPtr visualization_; //!< Instance of the visualization class
  VertexPose* vertex;
  RobotFootprintModelPtr robot_model_; //!< Robot model
  boost::shared_ptr<g2o::SparseOptimizer> optimizer_; //!< g2o optimizer for trajectory optimization

  bool initialized_; //!< Keeps track about the correct initialization of this class
  bool optimized_; //!< This variable is \c true as long as the last optimization has been completed successful
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
};

}

#endif