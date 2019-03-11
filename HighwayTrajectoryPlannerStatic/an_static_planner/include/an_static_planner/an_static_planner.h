/**
 * @file an_static_planner.h
 * @brief Defines the Planner_c class.
 * @authur: Tianhua Zhao
 */

#include <vector>
#include <ctime>
#include <chrono> 

#include "ros/ros.h"
#include "an_messages/lanes.h"
#include "an_messages/obstacles.h"
#include "geometry_msgs/PoseStamped.h"
#include "lib/OpenList.h"
#include "lib/Node.h"

/**
  * @brief Planning class, subscribes map, pose, goal, obstacle, path plan,
  * and publishes planner trajectory.
  */
class Planner_c {
public: 
  bool Init_(void);
  void Loop_(void);

  static constexpr double DEFAULT_LOOP_RATE = 150;
  static constexpr double DEFAULT_GRID_LENGTH = 20.0;
  static constexpr double DEFAULT_GRID_WIDTH = 3.7;
  static constexpr double DEFAULT_EPSILON = 1.1;
  double loop_rate_;
  double grid_length_;
  double grid_width_;
  double epsilon_;
private:
  //callbacks
  void lanes_callback_(const an_messages::lanes::ConstPtr& msg);
  void pose_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void goal_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void obstacles_callback_(const an_messages::obstacles::ConstPtr& msg);

  //helper functions
  std::vector<an_messages::trajectory> ReadMotionPrimitvesFromFile();
  void process_data_();
  std::vector<int> compute_path_();
  an_messages::trajectory motion_primitives_to_trajectory_(const std::vector<int>& motion_primitives);
  int x_y_to_grid_id_(double x, double y);
  void convert_obstacle_to_grid_(double x, double y, double width, double length);
  double get_heuristic_(int i);
  std::vector<std::pair<int/*node id*/, double/*cost*/> > get_successors_(int id);
  int get_motion_primitive_id_(int start, int end);

  //helper struct
  struct Obstacle_s {
    double x;
    double y;
    double width;
    double length;
    double theta;
  };

  //flags
  bool new_plan_;
  bool lanes_received_;
  bool pose_received_;
  bool goal_received_;
  bool obstacles_received_;

  //raw data received from msg
  int num_of_lanes_;
  double lane_width_;
  double x_begin_; //x of map begin
  double x_end_;  //x of map end
  double x_start_;  //start location
  double y_start_;
  double x_goal_;   //goal location
  double y_goal_;
  std::vector<Obstacle_s> obs_;

  //processed data
  int m_max_; //grid x dimension
  int n_max_; //grid y dimension
  int num_of_grid_;
  int start_id_; 
  int goal_id_;  
  std::vector<Node> nodes_;
  std::vector<bool> obstacles_;  

  // motion primitives
  std::vector<an_messages::trajectory> mps_;

  //ros
  ros::NodeHandle nh_;
  ros::Subscriber lanes_sub_, pose_sub_, goal_sub_, obstacles_sub_;
  ros::Publisher  planner_trajectory_pub_;
};
