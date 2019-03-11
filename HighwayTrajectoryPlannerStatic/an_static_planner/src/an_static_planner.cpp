#include "an_static_planner/an_static_planner.h"

bool Planner_c::Init_(void) {
    ROS_DEBUG("[planner] Entering init");

    loop_rate_ = DEFAULT_LOOP_RATE;
    grid_length_ = DEFAULT_GRID_LENGTH;
    grid_width_ = DEFAULT_GRID_WIDTH;
    epsilon_ = DEFAULT_EPSILON;

    new_plan_ = true;
    lanes_received_ = false;
    pose_received_ = false;
    goal_received_ = false;
    obstacles_received_ = false;

    num_of_lanes_ = 0;
    lane_width_ = 0;
    x_begin_ = 0;
    x_end_ = 0;
    x_start_ = 0;
    y_start_ = 0;
    x_goal_ = 0;
    y_goal_ = 0;
    obs_.clear();

    m_max_ = 0;
    n_max_ = 0;
    num_of_grid_ = 0;
    start_id_ = -1;
    goal_id_ = -1;
    nodes_.clear();
    obstacles_.clear();

    mps_ = ReadMotionPrimitvesFromFile();

    lanes_sub_ = nh_.subscribe("lanes", 1, &Planner_c::lanes_callback_, this);
    pose_sub_ = nh_.subscribe("pose", 1, &Planner_c::pose_callback_, this);
    goal_sub_ = nh_.subscribe("goal", 1, &Planner_c::goal_callback_, this);
    obstacles_sub_ = nh_.subscribe("obstacles", 1, &Planner_c::obstacles_callback_, this);
    
    planner_trajectory_pub_ = nh_.advertise<an_messages::trajectory>("planner_trajectory", 5, true);
    
    ROS_DEBUG("[planner] Init returned true");
    return true;
}

void Planner_c::Loop_(void) {
    ros::Rate loop(loop_rate_);
    while (ros::ok() ) {
        if (new_plan_ && pose_received_ && goal_received_
            && obstacles_received_ && lanes_received_) { 
                process_data_();
                new_plan_ = false;
                ROS_INFO("[planner] Started planning");               
                auto start = std::chrono::system_clock::now(); // clock starts
                std::vector<int> motion_primitives = compute_path_();
                an_messages::trajectory traj = motion_primitives_to_trajectory_(motion_primitives);
                auto end = std::chrono::system_clock::now();  // clock ends
                std::chrono::duration<double> elapsed_seconds = end-start;  // planning time
                ROS_INFO("[planner] Finished planning, planning time = %f sec", elapsed_seconds.count());
                ROS_DEBUG("[planner] Publishing planner trajectory");
                planner_trajectory_pub_.publish(traj);
        }
        ros::spinOnce();
        loop.sleep();
    }
}

void Planner_c::lanes_callback_(const an_messages::lanes::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering lanes callback");
    // process message
    num_of_lanes_ = msg->lanes.size();
    lane_width_ = msg->lanes.front().width;
    x_begin_ = msg->lanes.front().centerline.front().x;
    x_end_ = msg->lanes.front().centerline.back().x;

    lanes_received_ = true;
    ROS_DEBUG("[planner] Received new lanes data");
}

void Planner_c::pose_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering pose callback");
    // process message
    x_start_ = msg->pose.position.x;  
    y_start_= msg->pose.position.y;  

    pose_received_ = true;
    ROS_DEBUG("[planner] Received new pose data");
}

void Planner_c::goal_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering goal callback");
    // process message
    x_goal_= msg->pose.position.x;  
    y_goal_ = msg->pose.position.y;  

    goal_received_ = true;
    ROS_DEBUG("[planner] Received new goal data");
}

void Planner_c::obstacles_callback_(const an_messages::obstacles::ConstPtr& msg){
    ROS_DEBUG("[planner] Entering obstacle callback");
    obs_.reserve(msg->obs.size());
    // process message
    obstacles_.resize(num_of_grid_);
    for (an_messages::obstacle ob: msg->obs){
        geometry_msgs::Pose2D po = ob.path.front().traj.front().position;
        obs_.emplace_back(Obstacle_s{po.x, po.y, ob.width,ob.length,po.theta});
    }

    obstacles_received_ = true;
    ROS_DEBUG("[planner] Received new obstacle data");
}

std::vector<an_messages::trajectory> Planner_c::ReadMotionPrimitvesFromFile() {
  std::vector<an_messages::trajectory> mps;
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
    mps.clear();
    return mps;
  }

  int sz = fscanf(fp, "%i", &num_mps);
  if (sz == 0) {
    ROS_WARN("[planning] Error with motion primitve file: missing number of primitves");
    mps.clear();
    return mps;
  }

  mps.resize(num_mps);

  for (int mp_idx =-0; mp_idx < num_mps; mp_idx++) {
    int num_traj_pts=0;
    sz = fscanf(fp, "%i", &num_traj_pts);
    if (sz == 0) {
        ROS_WARN("[planning] Error with motion primitve file: missing number of trajectory points");
        mps.clear();
        return mps;
    }

    mps[mp_idx].traj.resize(num_traj_pts);

    for (int traj_pt_idx=0; traj_pt_idx < num_traj_pts; traj_pt_idx++) {
      an_messages::traj_pt pt;
      double sec_from_start;
      sz = fscanf(fp, "%lf %lf %lf %lf %lf %lf", &sec_from_start, &pt.position.x, &pt.position.y, &pt.position.theta, &pt.velocity.linear.x, &pt.velocity.linear.y);
      if (sz != 6) {
        ROS_WARN("[planning] Error with motion primitive file : missing trajectory point data");
        mps.clear();
        return mps;
      }
      pt.header.stamp = ros::Time(sec_from_start);
      mps[mp_idx].traj[traj_pt_idx] = pt;
    }
  }
  fclose(fp);
  return mps;
}

void Planner_c::process_data_(){
    //map
    m_max_ = (x_end_ - x_begin_) /  grid_length_ + 1;
    n_max_ = num_of_lanes_ * lane_width_ / grid_width_;
    num_of_grid_ = m_max_ * n_max_;
    nodes_.clear();
    nodes_.reserve(num_of_grid_);
    for(unsigned i = 0; i < num_of_grid_; i++){
       nodes_.emplace_back(i);
    }
    obstacles_.clear();
    obstacles_.resize(num_of_grid_);
    //start
    start_id_ = x_y_to_grid_id_(x_start_, y_start_);
    if(start_id_ == -1){
        ROS_ERROR("Invalid start location");
    }
    //goal
    goal_id_ = x_y_to_grid_id_(x_goal_, y_goal_);
    if(goal_id_ == -1){
        ROS_ERROR("Invalid goal location");
    }
    //obstacles
    for (auto ob : obs_){
        if(ob.theta != 0){
            ROS_WARN("Obstacle's theta is not 0, collision may occur");
        }
        convert_obstacle_to_grid_(ob.x, ob.y, ob.width, ob.length);
    }
}

std::vector<int> Planner_c::compute_path_(){
    // open list, contains generated nodes
	OpenList open(num_of_grid_);
	// closed list, contains expanded nodes
	std::vector<Node *> closed;
	closed.reserve(num_of_grid_);
	// series of motion primitives needed from start to goal, in reverse order
	std::vector<int> motion_primitives;

	Node *current_node = &nodes_[start_id_];
	// set g value
	current_node->SetG(0);
	// add to the open list
	open.push(current_node);
	current_node->SetType(OPEN);
	while (!open.empty())
	{
		// get the node with the min f value.
		current_node = open.top();
		if (current_node->GetId() == goal_id_)
		{
			// make path points from the current location(exclusive) to the goal location(exclusive)
			while (current_node->GetParentId() != -1)
			{
                int motion_primitive_id = get_motion_primitive_id_(current_node->GetParentId(), current_node->GetId());
				motion_primitives.push_back(motion_primitive_id);
				current_node = &nodes_[current_node->GetParentId()];
			}
			break;
		}
        open.pop();
		// add the node to the closed list
		closed.push_back(current_node);
		current_node->SetType(CLOSED);
		// expand the current node
        std::vector<std::pair<int/*node id*/, double/*cost*/> > neighbors = get_successors_(current_node->GetId());
        for (auto neighbor : neighbors){
            // generate the neighbor node
            Node* nptr = &nodes_[neighbor.first];
            double tentative_g = current_node->GetG() + neighbor.second;
            if(nptr->GetType() == DEFAULT){
                nptr->SetH(get_heuristic_(neighbor.first));
                nptr->SetG(tentative_g);
                nptr->SetParentId(current_node->GetId());
                open.push(nptr);
                nptr->SetType(OPEN);
            } else if (nptr->GetType() == OPEN){
                if(nptr->GetG() > tentative_g){
                    nptr->SetG(tentative_g);
                    open.decrease_key(nptr);
                    nptr->SetParentId(current_node->GetId());
                }
            }
        }
	}
	if (motion_primitives.empty())
	{
		ROS_INFO("[planning] No path is found.");
	}
	else
	{
		ROS_INFO("[planning] Successfully found a path.");
	}
	return motion_primitives;
}

an_messages::trajectory Planner_c::motion_primitives_to_trajectory_(const std::vector<int>& motion_primitives){
    an_messages::trajectory res;
    res.traj.reserve(motion_primitives.size() * mps_[1].traj.size());
    ros::Duration time_offset = ros::Duration(ros::Time::now().toSec() + 7);
    double x_offset = 0;
    double y_offset = 0;
    int seq = 0;
    // start traj pt
    an_messages::traj_pt mp_pt = mps_[motion_primitives[motion_primitives.size() - 1]].traj[0];
    an_messages::traj_pt pt;

    pt.header.seq = seq;
    seq++;
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

    // rest
    for (int i = motion_primitives.size() - 1; i >= 0; i--){
        int mp_idx = motion_primitives[i];
        for(int j =  1; j < mps_[mp_idx].traj.size(); j++){
            an_messages::traj_pt mp_pt = mps_[mp_idx].traj[j];
            an_messages::traj_pt pt;

            pt.header.seq = seq;
            seq++;
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
        time_offset += ros::Duration(mps_[mp_idx].traj.back().header.stamp.toSec());
        x_offset += mps_[mp_idx].traj.back().position.x;
        y_offset += mps_[mp_idx].traj.back().position.y;
    }
    return res;
}

int Planner_c::x_y_to_grid_id_(double x, double y){
    int m = (x + grid_length_ / 2) / grid_length_;
    int n = (y + grid_width_ / 2) / grid_width_;
    int id =  m * n_max_ + n;
    if(id < num_of_grid_){
        return id;
    }
    return -1;
}


void Planner_c::convert_obstacle_to_grid_(double x, double y, double width, double length){
    double x_min = x - length / 2;
    double x_max = x + length / 2;
    double y_min = y - width / 2;
    double y_max = y + width / 2;
    int m_min = (x_min + grid_length_ / 2) / grid_length_;
    int m_max = (x_max + grid_length_ / 2) / grid_length_;
    int n_min = (y_min + grid_width_ / 2)/ grid_width_;
    int n_max = (y_max + grid_width_ / 2)/ grid_width_;
    int m_cur = m_min;
    while(m_cur <= m_max){
        int n_cur = n_min;
        while(n_cur <= n_max){
            obstacles_[m_cur * n_max_ + n_cur] = true;
            ROS_DEBUG("Mark grid index %d as obstacle", m_cur * n_max_ + n_cur);
            n_cur++;
        }
        m_cur ++;
    }
}

double Planner_c::get_heuristic_(int i){
    double x = i / n_max_ * grid_length_;
    double y = i % n_max_ * grid_width_;
    double x_goal = goal_id_ / n_max_ * grid_length_;
    double y_goal = goal_id_ % n_max_ * grid_width_;
    // euclidean distance
    double delta_x = x - x_goal;
    double delta_y = y - y_goal;
    double dist  = std::pow(delta_x, 2) + std::pow(delta_y, 2);
    dist = std::sqrt(dist);
	return dist;
}

std::vector<std::pair<int/*node id*/, double/*cost*/> > Planner_c::get_successors_(int id){
    std::vector<std::pair<int, double> > res;
    bool is_valid = true;
    // straight
    std::vector<int> grid_coverage_straight /* precomputed and preset grid coverage */ = 
                                        {id, id + n_max_, id + n_max_ * 2,
                                         id + n_max_ * 3, id + n_max_ * 4,
                                        id + n_max_ * 5};
    for (int idx : grid_coverage_straight){
        if(idx >= num_of_grid_ || obstacles_[idx]){
            is_valid = false;
            break;
        }    // boundary check, collision check
    }
    if (is_valid){
        res.emplace_back(id + n_max_ * 5, 100);
    } else {
        is_valid = true;  // reset
    }
    // left
    if((id + 1) % n_max_ != 0){ // boundary check
        std::vector<int> grid_coverage_left /* precomputed and preset grid coverage */ = 
                                            {id, id + n_max_, id + n_max_ * 2, id + n_max_ * 2 + 1,
                                            id + n_max_ * 3, id + n_max_ * 3 + 1, id + n_max_ * 4 + 1,
                                            id + n_max_ * 5 + 1};
        for (int idx : grid_coverage_left){
            if(idx >= num_of_grid_ || obstacles_[idx]){
                is_valid = false;
                break;
            }    // boundary check, collision check
        }
        if (is_valid){
            res.emplace_back(id + n_max_ * 5 + 1, 105);
        } else {
            is_valid = true;  // reset
        }
    }
    // right
    if(id % n_max_ != 0){ // boundary check
        std::vector<int> grid_coverage_right /* precomputed and preset grid coverage */ = 
                                            {id, id + n_max_, id + n_max_ * 2, id + n_max_ * 2 - 1,
                                            id + n_max_ * 3, id + n_max_ * 3 - 1, id + n_max_ * 4 - 1,
                                            id + n_max_ * 5 - 1};
        for (int idx : grid_coverage_right){
            if(idx >= num_of_grid_ || obstacles_[idx]){
                is_valid = false;
                break;
            }    // boundary check, collision check
        }
        if (is_valid){
            res.emplace_back(id + n_max_ * 5 - 1, 105);
        } else {
            is_valid = true;  // reset
        }
    }
    return res;
}

int Planner_c::get_motion_primitive_id_(int start, int end){
    int lane_start = start % n_max_;
    int lane_end = end % n_max_;
    if(lane_start == lane_end){
        return 0;
    }
    else if(lane_start > lane_end){
        return 2;
    }
    else {
        return 1;
    }
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