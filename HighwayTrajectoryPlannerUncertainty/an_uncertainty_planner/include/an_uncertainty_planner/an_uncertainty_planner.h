/**
 * @file an_uncertainty_planner.h
 * @brief Defines the Planner_c class.
 * @authur: Tianhua Zhao
 */

#include <vector>
#include <ctime>
#include <chrono> 
#include <time.h>
#include <limits>
#include <queue>

#include "ros/ros.h"
#include "an_messages/lanes.h"
#include "an_messages/obstacles.h"
#include "an_messages/observation.h"
#include "geometry_msgs/PoseStamped.h"
#include "lib/HelperStructs.h"
#include "lib/CollisionChecker.h"

/**
  * @brief Planning class, subscribes map, pose, goal, obstacle, observation,
  * and publishes policy step.
  */
class Planner_c {
public: 
  bool Init_(void);
  void Loop_(void);
  // subscriber loop rate
  double loop_rate_ = 1000;  
  //discretization
  double grid_length_ = 10;
  double grid_width_ = 3.7;
  double t_interval_ = 0.2;
  // ego dimension
  double ego_length_ = 4.8;
  double ego_width_ = 1.8;
  // ego speed, used for approximating max time
  double ego_v = 25.0;
  // delay to start ego
  double time_delay_ = 6.0; 
  // number of intents of the obstacle
  double num_of_intents_ = 2;
  // flash headlight range
  double flash_range_min_ = 30;
  double flash_range_max_ = 50;
  double v_i_time_allowed_ = 1; // available time for value iteration
  double v_i_delta = 1; // delta for value iteration
private:
  // callbacks
  void LaneCallback(const an_messages::lanes::ConstPtr& msg);
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void ObstacleCallback(const an_messages::obstacles::ConstPtr& msg);
  void ObservationCallback(const an_messages::observation::ConstPtr& msg);
  // read motion primitives from file
  void ReadMotionPrimitvesFromFile();
  // before generating policy, process raw data
  void ProcessData();
  // compute cell coverages for an obstacle, i.e., comvert an obstacle to lower resolution repesentation
  void ComputeObstacleCellCoverage(Obstacle_s ob);
  // compute relative cell coverages for motion primitives
  void ComputeMotionPrimitiveCellCoverage();
  // update boolean array coverage map using given vehicle configuration
  void UpdateCoverageMap(std::vector<bool> & coverage_map, double x, double y, double t, double theta, double length, double width, double half_diagonal, double theta_diag);
  // generate policy
  void GeneratePolicy(); 
  // build graph nodes and edges
  void BuildGraph();
  // get successors of the state with given id
  std::vector<Successor_s> GetSuccessors(int id);
  // from given n value, execute given motion primitive, is the ending state out of bound?
  bool OutOfBound(int n, int mp_id);
  // from the given state, execute given motion primitve, in collision with obstacle?
  bool InCollision(int start_id, int mp_id);
  // is the given state in valid range for flashing headlight?
  bool InRange(int id);
  // value iteration, stop when time runs out or all bellman erros are less than delta
  void ValueIteration(double time_allowed, double delta);
  // given x, y, t, and next action, generate trajectory
  an_messages::trajectory GetTrajectory(int next_action_id, double x, double y, double t);
  // get node id
  int GetNodeID(double x, double y, double t);
  int GetNodeID(double x, double y, double t, double intent);
  // get x, y, t and intention from state id
  Coordinates_s GetCoordinates(int id);
  // flags
  bool new_policy_;
  bool lanes_received_;
  bool pose_received_;
  bool goal_received_;
  bool obstacles_received_;
  bool observation_received_;
  bool policy_available_;
  // raw data received from msg
  int num_of_lanes_;
  double lane_width_;
  double x_begin_; //x of map begin
  double x_end_;  //x of map end
  double x_start_;  //start location
  double y_start_;
  double x_goal_;   //goal location
  double y_goal_;
  std::vector<Obstacle_s> obs_;  // obstacle info
  Observation_s observ_;  // observation info
  double flash_cost_;  // cost to flash headlight
  // processed data
  int m_max_; //grid x max
  int n_max_; //grid y max
  int T_max_;
  int num_of_nodes_;
  int start_id_; 
  int m_goal_;
  int n_goal_;  
  std::vector<Node_s> nodes_;
  std::vector<std::vector<bool> > obstacles_;  // obstacle info for 3 differnt beliefs
  // start time
  double start_time_;
  // motion primitives
  std::vector<MotionPrim_s> mps_;
  std::vector<std::vector<int> > mp_grid_coverage_;
  // states for value iteration
  std::vector<Node_s*> value_iteration_states_;
  // collision checker
  CollisionChecker col_checker_ = CollisionChecker();
  // ros
  ros::NodeHandle nh_;
  ros::Subscriber lanes_sub_, pose_sub_, goal_sub_, obstacles_sub_, observation_sub_;
  ros::Publisher  policy_step_pub_;
};
