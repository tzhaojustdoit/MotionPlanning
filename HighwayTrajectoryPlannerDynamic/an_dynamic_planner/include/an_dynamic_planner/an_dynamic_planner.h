/**
 * @file an_dynamic_planner.h
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
  static constexpr double DEFAULT_GRID_LENGTH = 5;
  static constexpr double DEFAULT_GRID_WIDTH = 3.7;
  static constexpr double DEFAULT_EPSILON = 1.1;
  static constexpr double DEFAULT_T_INTERVAL = 0.2;
  static constexpr double DEFAULT_TIME_MAX = 80;
  static constexpr double TIME_DELAY = 4.5;
  static constexpr double DEFAULT_EGO_LENGTH = 4.8;
  static constexpr double DEFAULT_EGO_WIDTH = 1.8;  
  static constexpr double MP_0_Length = 5.0;
  static constexpr double MP_0_Duration = 0.2;
  static constexpr double MP_0_Cost = 5;
  static constexpr double MP_1_Length = 100;
  static constexpr double MP_1_Duration = 4;
  static constexpr double MP_1_Cost = 150;
  static constexpr double MP_2_Length = 100;
  static constexpr double MP_2_Duration = 4;
  static constexpr double MP_2_Cost = 150;

  double loop_rate_;
  double grid_length_;
  double grid_width_;
  double epsilon_;
  double t_interval_;
  double time_max_;
  double ego_length_;
  double ego_width_;
private:
  //callbacks
  void lanes_callback_(const an_messages::lanes::ConstPtr& msg);
  void pose_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void goal_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void obstacles_callback_(const an_messages::obstacles::ConstPtr& msg);
  
  //helper structs
  struct Traj_s {
    double x;
    double y;
    double theta;
    double secFromStart;
  };

  struct Obstacle_s {
    double width;
    double length;
    std::vector<Traj_s> traj;
  };

  struct Successor_s {
    int succesor_id;
    double cost;
    int mp_id;
  };

  struct Point_s {
    double x;
    double y;
  };

  struct Line_s {
    Point_s a;
    Point_s b;
  };

  //helper functions
  std::vector<an_messages::trajectory> ReadMotionPrimitvesFromFile();
  void compute_mp_grid_coverage();
  void process_data_();
  std::vector<int> compute_path_();
  an_messages::trajectory motion_primitives_to_trajectory_(const std::vector<int>& motion_primitives);
  int x_y_t_to_grid_id_(double x, double y, double t);
  void convert_obstacle_to_grid_(Obstacle_s);
  double get_heuristic_(int i);
  std::vector<Successor_s> get_successors_(int id);
  std::vector<int> get_grid_coverage_(int mpId, int id);
  bool boxCheck(Point_s, Point_s, Point_s, Point_s, Point_s, Point_s, Point_s, Point_s);
  bool intersection(Line_s line1, Line_s line2);
  int order(Line_s line1, Point_s pt);
  bool onSegment(Line_s line1, Point_s pt);
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
  int T_max_;
  int num_of_grid_;
  int start_id_; 
  int goal_id_;  
  std::vector<Node> nodes_;
  std::vector<bool> obstacles_;  
  double start_time_;
  // motion primitives
  std::vector<an_messages::trajectory> mps_;
  std::vector<std::vector<int> > mp_grid_coverage_;
  
  //ros
  ros::NodeHandle nh_;
  ros::Subscriber lanes_sub_, pose_sub_, goal_sub_, obstacles_sub_;
  ros::Publisher  planner_trajectory_pub_;
};
