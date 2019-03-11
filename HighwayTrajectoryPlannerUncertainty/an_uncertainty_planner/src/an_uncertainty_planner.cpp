#include "an_uncertainty_planner/an_uncertainty_planner.h"

bool Planner_c::Init_(void) {
    ROS_DEBUG("[planner] Entering init");
    // initialize flags
    new_policy_ = true;
    lanes_received_ = false;
    pose_received_ = false;
    goal_received_ = false;
    obstacles_received_ = false;
    observation_received_ = false;
    policy_available_ = false;
    // initialize raw data from msg
    num_of_lanes_ = 0;
    lane_width_ = 0;
    x_begin_ = 0;
    x_end_ = 0;
    x_start_ = 0;
    y_start_ = 0;
    x_goal_ = 0;
    y_goal_ = 0;
    obs_.clear();
    observ_ = Observation_s();
    // initialize processed data
    m_max_ = 0;
    n_max_ = 0;
    T_max_ = 0;
    num_of_nodes_ = 0;
    start_id_ = -1;
    m_goal_ = -1;
    n_goal_ = -1;
    nodes_.clear();
    obstacles_.clear();
    obstacles_.resize(3);
    value_iteration_states_.clear();
    // initialize nodes
    lanes_sub_ = nh_.subscribe("lanes", 1, &Planner_c::LaneCallback, this);
    pose_sub_ = nh_.subscribe("pose", 1, &Planner_c::PoseCallback, this);
    goal_sub_ = nh_.subscribe("goal", 1, &Planner_c::GoalCallback, this);
    obstacles_sub_ = nh_.subscribe("obstacles", 1, &Planner_c::ObstacleCallback, this);
    observation_sub_ = nh_.subscribe("observation", 1, &Planner_c::ObservationCallback, this);
    policy_step_pub_ = nh_.advertise<an_messages::trajectory>("policy_step", 5, true);
    // read motion primitives from file
    ReadMotionPrimitvesFromFile(); 
    col_checker_ = CollisionChecker();
    // read cost of flashing headlight
    while (!ros::param::has("flash_headlight_cost")) {
        ROS_WARN("[planning] sleeping while waiting for flash_headlight_cost");
        ros::Duration(0.1).sleep();
    }
    ros::param::get("/flash_headlight_cost", flash_cost_);
    ROS_DEBUG("[planning] flash headlight cost: %lf", flash_cost_);
    
    ROS_DEBUG("[planner] Init returned true");
    return true;
}

void Planner_c::Loop_(void) {
    ros::Rate loop(loop_rate_);
    while (ros::ok() ) {
        if (new_policy_ && pose_received_ && goal_received_
            && obstacles_received_ && lanes_received_) { 
                auto start = std::chrono::system_clock::now(); // clock starts
                ProcessData();
                new_policy_ = false;
                ROS_DEBUG("[planner] Generating policy");  
                GeneratePolicy();     // generate policy 
                policy_available_ = true;       
                auto end = std::chrono::system_clock::now();  // clock ends
                std::chrono::duration<double> elapsed_seconds = end-start;  // planning time
                ROS_DEBUG("[planner] Finished generating policy, planning time = %f sec", elapsed_seconds.count());
                int next_action = nodes_[start_id_].best_action;
                an_messages::trajectory traj = GetTrajectory(next_action, x_start_, y_start_, start_time_);
                ROS_DEBUG("[planner] Publishing initial policy step");
                policy_step_pub_.publish(traj);
                observation_received_ = false;
        }
        if (observation_received_ && policy_available_){
            int node_id = GetNodeID(observ_.x, observ_.y, observ_.t, observ_.change); // the node id for the (x, y, t, intention)
            if(node_id == -1){
                ROS_DEBUG("[planner] Location and time out of bound");
                break;
            }
            int next_action = nodes_[node_id].best_action;
            if(next_action == -1){
                break;
            }
            an_messages::trajectory traj = GetTrajectory(next_action, observ_.x, observ_.y, observ_.t);
            ROS_DEBUG("[planner] Publishing policy step");
            policy_step_pub_.publish(traj);
            observation_received_ = false;
        }
        ros::spinOnce();
        loop.sleep();
    }
}

void Planner_c::LaneCallback(const an_messages::lanes::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering lanes callback");
    // process message
    num_of_lanes_ = msg->lanes.size();
    lane_width_ = msg->lanes.front().width;
    x_begin_ = msg->lanes.front().centerline.front().x;
    x_end_ = msg->lanes.front().centerline.back().x;

    lanes_received_ = true;
    ROS_DEBUG("[planner] Received new lanes data");
}

void Planner_c::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering pose callback");
    // process message
    x_start_ = msg->pose.position.x;  
    y_start_= msg->pose.position.y;  

    pose_received_ = true;
    ROS_DEBUG("[planner] Received new pose data");
}

void Planner_c::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering goal callback");
    // process message
    x_goal_= msg->pose.position.x;  
    y_goal_ = msg->pose.position.y;  

    goal_received_ = true;
    ROS_DEBUG("[planner] Received new goal data");
}

void Planner_c::ObstacleCallback(const an_messages::obstacles::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering obstacle callback");
    obs_.reserve(msg->obs.size());
    // process message
    obstacles_.resize(num_of_nodes_);
    for (an_messages::obstacle ob: msg->obs){
        std::vector<Traj_s> trajs;
        for(int i = 0; i < ob.path.size(); i++){
            double probability = ob.probability[i];
            auto traj_msg = ob.path[i].traj;
            std::vector<TrajPt_s> traj_pts;
            for(auto traj_pt : traj_msg)
            {
                auto position = traj_pt.position;
                double t = traj_pt.header.stamp.toSec();
                traj_pts.emplace_back(TrajPt_s{position.x, position.y, position.theta, t});
            }
            trajs.emplace_back(Traj_s{traj_pts, probability});
        }
        obs_.emplace_back(Obstacle_s{ob.length, ob.width, trajs});
    }

    obstacles_received_ = true;
    ROS_DEBUG("[planner] Received new obstacle data");
}

void Planner_c::ObservationCallback(const an_messages::observation::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering observation callback");
    // process message
    double t = msg->header.stamp.toSec();
    double change = msg->Change;
    double x = msg->x;
    double y = msg->y;
    if(t == observ_.t && change == observ_.change && x == observ_.x && y == observ_.y){
        return;
    }
    observ_.t = t;
    observ_.change = change;
    observ_.x = x;
    observ_.y = y;
    observation_received_ = true;
    ROS_DEBUG("[planner] Received new observation data");
}

void Planner_c::ReadMotionPrimitvesFromFile() {
  int num_mps = 0;

  std::string fname;
  while (!ros::param::has("MPRIM_FILE")) {
    ROS_WARN("[planning] sleeping while waiting");
    ros::Duration(0.1).sleep();
  }

  ros::param::get("/MPRIM_FILE", fname);
  ROS_DEBUG("[planning] MP: %s", fname.c_str() );

  FILE* fp = fopen(fname.c_str(), "r");
  if (fp == NULL) {
    ROS_WARN("[planning] No motion primitive file!  Exiting!!");
    mps_.clear();
    return;
  }

  int sz = fscanf(fp, "%i", &num_mps);
  if (sz == 0) {
    ROS_WARN("[planning] Error with motion primitve file: missing number of primitves");
    mps_.clear();
    return;
  }

  mps_.resize(num_mps);

  for (int mp_idx = 0; mp_idx < num_mps; mp_idx++) {
    double length;
    double duration;
    double cost;
    int num_traj_pts = 0;
    sz = fscanf(fp, "%lf %lf %lf %i", &length, &duration, &cost, &num_traj_pts);
    if (sz == 0) {
        ROS_WARN("[planning] Error with motion primitve file: missing length, duration, cost or number of trajectory points");
        mps_.clear();
        return;
    }
    an_messages::trajectory traj;
    traj.traj.resize(num_traj_pts);
    for (int traj_pt_idx=0; traj_pt_idx < num_traj_pts; traj_pt_idx++) {
      an_messages::traj_pt pt;
      double sec_from_start;
      sz = fscanf(fp, "%lf %lf %lf %lf %lf %lf", &sec_from_start, &pt.position.x, &pt.position.y, &pt.position.theta, &pt.velocity.linear.x, &pt.velocity.linear.y);
      if (sz != 6) {
        ROS_WARN("[planning] Error with motion primitive file : missing trajectory point data");
        mps_.clear();
        return;
      }
      pt.header.stamp = ros::Time(sec_from_start);
      traj.traj[traj_pt_idx] = pt;
    }
    mps_[mp_idx] = MotionPrim_s{length, duration, cost, traj};
  }
  fclose(fp);
}

void Planner_c::ProcessData(){
    // start time
    start_time_ = ros::Time::now().toSec() + time_delay_;
    // map
    m_max_ = (x_end_ - x_begin_) /  grid_length_ + 1;
    n_max_ = num_of_lanes_ * lane_width_ / grid_width_;
    T_max_ = x_end_ / ego_v / t_interval_;
    num_of_nodes_ = m_max_ * n_max_ * T_max_;
    nodes_.clear();
    nodes_.resize(num_of_nodes_ * 3);
    // start
    start_id_ = GetNodeID(x_start_, y_start_, start_time_);
    // goal
    m_goal_ = (x_goal_ + grid_length_ / 2) / grid_length_;
    n_goal_ = (y_goal_ + grid_width_ / 2) / grid_width_;
    // obstacles
    obstacles_.clear();
    obstacles_.resize(num_of_intents_);
    for (auto item : obstacles_){
        item.resize(num_of_nodes_);
    }
    for (auto ob : obs_){
        ComputeObstacleCellCoverage(ob);
    }
    // computer motion primitive coverage
    ComputeMotionPrimitiveCellCoverage();
    // set ending state offsets
    mps_[0].ending_state_id_offset = int (mps_[0].length / grid_length_) * n_max_ 
                            + int(mps_[0].duration / t_interval_) * n_max_ * m_max_;
    mps_[1].ending_state_id_offset = 1 + int(mps_[1].length / grid_length_) * n_max_ 
                            + int(mps_[1].duration / t_interval_) * n_max_ * m_max_;
    mps_[2].ending_state_id_offset = -1 + int(mps_[2].length / grid_length_) * n_max_
                             + int(mps_[2].duration / t_interval_) * n_max_ * m_max_;                        
}

void Planner_c::ComputeObstacleCellCoverage(Obstacle_s ob){
    double length = ob.length;
    double width = ob.width;
    double half_diagonal = sqrt(length* length + width * width)/2;
    double theta_diag = std::atan(width / length);
    for(int i = 0; i < ob.trajs.size(); i++){
        std::vector<bool> coverage_map(num_of_nodes_, false);
        for (TrajPt_s traj_pt : ob.trajs[i].traj){
            double x = traj_pt.x;
            double y = traj_pt.y;
            double theta = traj_pt.theta;
            double t = traj_pt.secFromStart - start_time_;
            UpdateCoverageMap(coverage_map, x, y, t, theta, length, width, half_diagonal, theta_diag);
        }
        obstacles_[i] = coverage_map;
    }
}

void Planner_c::ComputeMotionPrimitiveCellCoverage(){
    mp_grid_coverage_.resize(mps_.size());
    double half_diagonal = sqrt(ego_length_* ego_length_ + ego_width_ * ego_width_)/2;
    double theta_diag = std::atan(ego_width_ / ego_length_);
    for(int i = 0; i < mps_.size(); i++){     
        std::vector<int> coverageIdx;
        std::vector<bool> coverage_map(num_of_nodes_, false);
        for(an_messages::traj_pt item : mps_[i].traj.traj){
            double x = item.position.x;
            double y = item.position.y + grid_width_;
            double theta = item.position.theta;
            double t = item.header.stamp.toSec();
            UpdateCoverageMap(coverage_map, x, y, t, theta, ego_length_, ego_width_, half_diagonal, theta_diag);
        }
        for(int j = 0; j < coverage_map.size(); j++){
            if(coverage_map[j]){
                coverageIdx.push_back(j);
            }
        }
        mp_grid_coverage_[i] = coverageIdx;
    }
}

void Planner_c::UpdateCoverageMap(std::vector<bool> & coverage_map, double x, double y, 
                                    double t, double theta, double length, double width, 
                                    double half_diagonal, double theta_diag){
    // ego box
    double theta_sum = theta_diag + theta;
    double delta_x = half_diagonal * std::cos(theta_sum);
    double delta_y = half_diagonal * std::sin(theta_sum);
    Point_s ego_Top_left{x + delta_x, y + delta_y};
    Point_s ego_bottom_right{x - delta_x, y - delta_y};                        
    double theta_sub = theta - theta_diag;
    delta_x = half_diagonal * std::cos(theta_sub);
    delta_y = half_diagonal * std::sin(theta_sub);
    Point_s ego_Top_right{x + delta_x, y + delta_y};
    Point_s ego_bottom_left{x - delta_x, y - delta_y}; 
    // for outer circle check
    double max_possible_x = x + half_diagonal;
    double min_possible_x = x - half_diagonal;
    double max_possible_y = y + half_diagonal;
    double min_possible_y = y - half_diagonal;
    int m_min = (min_possible_x + grid_length_ / 2) / grid_length_;
    int m_max = (max_possible_x + grid_length_ / 2) / grid_length_;
    int n_min = (min_possible_y + grid_width_ / 2)/ grid_width_;
    int n_max = (max_possible_y + grid_width_ / 2)/ grid_width_;
    // for inner circle check
    double max_inner_x = x + width / 2;
    double min_inner_x = x - width / 2;
    double max_inner_y = y + width / 2;
    double min_inner_y = y - width / 2; 
    int m_min_inner = (max_inner_x + grid_length_ / 2) / grid_length_;
    int m_max_inner = (max_inner_x + grid_length_ / 2) / grid_length_;
    int n_min_inner = (min_inner_y + grid_width_ / 2)/ grid_width_;
    int n_max_inner = (max_inner_y + grid_width_ / 2)/ grid_width_;    
    if(n_min < 0){
        n_min = 0;
    }
    if(n_min >= n_max_){
        n_max = n_max_ - 1;
    }
    if(m_min < 0){
        m_min = 0;
    }
    if(m_max > m_max_){
        m_max = m_max_ - 1;
    }
    int T = (t + t_interval_ / 2) / t_interval_;          
    int m_cur = m_min;
    while(m_cur <= m_max){
        int n_cur = n_min;
        while(n_cur <= n_max){
            int idx = T * m_max_ * n_max_ + m_cur * n_max_ + n_cur;
            if (idx >= 0 && idx < num_of_nodes_){
                // inner check
                if(m_cur >= m_min_inner && m_cur <= m_max_inner && n_cur >= n_min_inner && n_cur <= n_max_inner){
                    coverage_map[idx] = true;
                } else {                          
                    // grid box
                    Point_s grid_Top_left {m_cur * grid_length_ + grid_length_ / 2, n_cur * grid_width_ + grid_width_ / 2};
                    Point_s grid_Top_right {m_cur * grid_length_ + grid_length_ / 2, n_cur * grid_width_ - grid_width_ / 2};
                    Point_s grid_bottom_left {m_cur * grid_length_ - grid_length_ / 2, n_cur * grid_width_ + grid_width_ / 2};
                    Point_s grid_bottom_right {m_cur * grid_length_ - grid_length_ / 2, n_cur * grid_width_ - grid_width_ / 2};
                    // bounding box check
                    bool inCollision = col_checker_.BoundingBoxCheck(grid_Top_left, grid_Top_right, grid_bottom_left, grid_bottom_right, 
                                                ego_Top_left, ego_Top_right, ego_bottom_left, ego_bottom_right);
                    if(inCollision){
                        coverage_map[idx] = true;
                    }
                }
            }
            n_cur++;
        }
        m_cur ++;
    }
}

// bool Planner_c::CompareX(int i, int j){
//     return nodes_[i].x > nodes_[j].x;
// }

void Planner_c::ValueIteration(double time_allowed, double tolerance){
    // sort by x descending
    std::sort(value_iteration_states_.begin(), value_iteration_states_.end(), [](Node_s * first, Node_s * second) { return first->x > second->x; });
    ROS_DEBUG("[planner] Value iterating over %i states", (int)value_iteration_states_.size());
    time_t cur;
    time(&cur);  // current time
    time_t deadline = cur + time_allowed; 
    double delta_max = std::numeric_limits<double>::infinity();
    int counter = 0;   
    while(/*cur < deadline &&*/ delta_max > tolerance){
        counter++;
        delta_max = 0;
        // one value iteration
        for (Node_s * cur_node : value_iteration_states_){   
            if(cur_node->successors.size() == 0){
                continue;
            } 
            double min_v = std::numeric_limits<double>::infinity(); 
            int best_action = -1;
            for (Successor_s suc : cur_node->successors){                
                double tentative_v = 0;
                for (Possible_Ending_State_s st : suc.possible_ending_states){
                    tentative_v += (suc.action_cost + nodes_[st.ending_state_id].v) * st.probability;
                }
                if (tentative_v < min_v){
                    min_v = tentative_v;
                    best_action = suc.action_id;
                }
            }
            double prev = cur_node->v;
            cur_node->v = min_v;
            cur_node->best_action = best_action;
            double delta = std::abs(min_v - prev);
            if(delta > delta_max){
                delta_max = delta;
            }
        }
        time(&cur);
    }
    ROS_DEBUG("[planner] Finished value iteration, number of iterations: %i, final delta: %f", counter, delta_max);
}

std::vector<Successor_s> Planner_c::GetSuccessors(int id){
    std::vector<Successor_s> res;
    res.reserve(4);
    Coordinates_s coor = GetCoordinates(id);
    // go straight, left turn, right turn
    for(int i = 0; i < mps_.size(); i++)
    {
        // boundary check
        bool out_of_bound = OutOfBound(coor.n, i);
        if(out_of_bound){  
            continue;
        }
        // collision check
        bool in_collison = InCollision(id, i);
        if(in_collison){    
            continue;
        }
        Successor_s suc;
        suc.action_id = i;
        suc.action_cost = mps_[i].cost;  
        suc.possible_ending_states.push_back(Possible_Ending_State_s{id + mps_[i].ending_state_id_offset, 1});
        res.push_back(suc);
    }
    // flash
    if(coor.intent == 0 /*unsure*/ && InRange(id)){
        Successor_s suc;
        suc.action_id = 3;
        suc.action_cost = flash_cost_; 
        suc.possible_ending_states.push_back(Possible_Ending_State_s{id + num_of_nodes_, 
                                            obs_.front().trajs[0].probability}); // belief:stay
        suc.possible_ending_states.push_back(Possible_Ending_State_s{id + 2 * num_of_nodes_, 
                                            obs_.front().trajs[1].probability}); // belief:change
        res.push_back(suc);
    }

    return res;
}

bool Planner_c::OutOfBound(int n, int mp_id){
    if(mp_id == 1 && n == n_max_ - 1){
        return true;
    }
    if(mp_id == 2 && n == 0){
        return true;
    }
    return false;
}

bool Planner_c::InCollision(int start_id, int mp_id){
    int intent = start_id / num_of_nodes_;
    start_id = start_id % num_of_nodes_;  // without intent dimension
    std::vector<int> cell_coverage;
    for (int item : mp_grid_coverage_[mp_id]){
        cell_coverage.push_back(start_id + item - 1);
    }   
    if(intent == 0){
        for(int idx : cell_coverage){
            if(idx >= num_of_nodes_ || obstacles_[0][idx] || obstacles_[1][idx]){
                return true;
            }
        }
    }
    if(intent == 1){
        for(int idx : cell_coverage){
            if(idx >= num_of_nodes_ || obstacles_[0][idx]){
                return true;
            }
        }   
    }
    if(intent == 2){
        for(int idx : cell_coverage){
            if(idx >= num_of_nodes_ || obstacles_[1][idx]){
                return true;
            }
        }   
    }
    return false;
}

bool Planner_c::InRange(int id){
    int min = flash_range_min_ / grid_length_;
    int max = flash_range_max_ / grid_length_;
    int cur = min + 1;
    std::vector<int> cell_coverage;
    while(cur < max){
        cell_coverage.push_back(id + cur * n_max_);
        cur++;
    }
    for(int idx : cell_coverage){
        if(idx >= num_of_nodes_ || obstacles_[0][idx] || obstacles_[1][idx]){
            return true;
        }
    }  
    return false;     
}

Coordinates_s Planner_c::GetCoordinates(int id){
    Coordinates_s res;
    res.intent = id / num_of_nodes_;
    res.T = id % num_of_nodes_ / (m_max_ * n_max_);
    res.m = id % num_of_nodes_ % (m_max_ * n_max_) / n_max_;
    res.n = id % num_of_nodes_ % (m_max_ * n_max_) % n_max_;
    return res;
}

void Planner_c::BuildGraph(){
    for(unsigned i = 0; i < nodes_.size(); i++){
        nodes_[i].id = i;
    }
    // bfs open list, contains generated nodes
	std::queue<Node_s *> open;
	Node_s *current_node = &nodes_[start_id_];	
    // add to the open list
	open.push(current_node);
	current_node->type = OPEN;
    // bfs
	while (!open.empty())
	{
		current_node = open.front();
        open.pop();
		current_node->type = CLOSED;
        Coordinates_s coor = GetCoordinates(current_node->id);
        if(coor.m <= m_goal_){
            if(coor.m == m_goal_ && coor.n == n_goal_){
                current_node->v = 0;
            }else{
                current_node->x = coor.m * grid_length_;
                value_iteration_states_.push_back(current_node);  
            }                       
        }
        std::vector<Successor_s> successors = GetSuccessors(current_node->id);
        current_node->successors = successors;
        // generate successor nodes
        for (Successor_s suc : successors){
            for (Possible_Ending_State_s st : suc.possible_ending_states){            
                Node_s* nptr = &nodes_[st.ending_state_id];
                if(nptr->type == DEFAULT){
                    open.push(nptr);
                    nptr->type = OPEN;
                }                  
            }
        }  
	}
    // sort value iteration states by x values descending
}

void Planner_c::GeneratePolicy(){
    BuildGraph();
    ValueIteration(v_i_time_allowed_, v_i_delta);
}

an_messages::trajectory Planner_c::GetTrajectory(int next_action_id, double x_offset, double y_offset, double t){
    ros::Duration time_offset = ros::Duration(t);
    an_messages::trajectory res;
    if(next_action_id == 3){ // flash
        an_messages::traj_pt pt;  // absolute traj point
        pt.header.stamp = ros::Time(t);
        pt.header.frame_id = "/map";

        pt.position.x = x_offset;
        pt.position.y = y_offset;
        pt.position.theta = 0;

        pt.velocity.linear.x =  -1;
        pt.velocity.linear.y =  0;
        pt.velocity.linear.z = 0;

        pt.velocity.angular.x = 0;
        pt.velocity.angular.y = 0;
        pt.velocity.angular.z = 0;

        pt.accel.linear.x =  0;
        pt.accel.linear.y =  0;
        pt.accel.linear.z = 0;

        pt.accel.angular.x = 0;
        pt.accel.angular.y = 0;
        pt.accel.angular.z = 0;
        res.traj.push_back(std::move(pt));
        return res;
    }
    res.traj.reserve(mps_[next_action_id].traj.traj.size());
    for(int j =  0; j < mps_[next_action_id].traj.traj.size(); j++){
        an_messages::traj_pt mp_pt = mps_[next_action_id].traj.traj[j]; // relative traj point 
        an_messages::traj_pt pt;  // absolute traj point

        pt.header.stamp = mp_pt.header.stamp + time_offset;
        pt.header.frame_id = "/map";

        pt.position.x = mp_pt.position.x + x_offset;
        pt.position.y = mp_pt.position.y + y_offset;
        pt.position.theta = mp_pt.position.theta;

        pt.velocity.linear.x =  mp_pt.velocity.linear.x;
        pt.velocity.linear.y =  mp_pt.velocity.linear.y;
        pt.velocity.linear.z = 0;

        pt.velocity.angular.x = 0;
        pt.velocity.angular.y = 0;
        pt.velocity.angular.z = 0;

        pt.accel.linear.x =  0;
        pt.accel.linear.y =  0;
        pt.accel.linear.z = 0;

        pt.accel.angular.x = 0;
        pt.accel.angular.y = 0;
        pt.accel.angular.z = 0;

        res.traj.push_back(std::move(pt));
    }
    return res;
}

int Planner_c::GetNodeID(double x, double y, double t){
    int m = (x + grid_length_ / 2) / grid_length_;
    int n = (y + grid_width_ / 2) / grid_width_;
    int T = (t - start_time_ + t_interval_ / 2) / t_interval_;
    int id =  m_max_ * n_max_ * T + m * n_max_ + n;
    if(id < num_of_nodes_){
        return id;
    }
    return -1;
}

int Planner_c::GetNodeID(double x, double y, double t, double intent){
    int id = GetNodeID(x, y, t);
    if(id == -1){
        return -1;
    }
    if(intent == 0){
        id += num_of_nodes_;
    }else if(intent == 1){
        id += num_of_nodes_ * 2;
    }
    return id;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    ROS_DEBUG("[planner] Starting");
    Planner_c planner;
    if (planner.Init_())
    {
        ROS_DEBUG("[planner] Entering loop");
        planner.Loop_();
    }
}