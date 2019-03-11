/**
 * @file HelperStucts.h
 * @brief Defines helper structs
 * @authur: Tianhua Zhao
 */

#ifndef HELPER_STRUCTS_
#define HELPER_STRUCTS_

#include <vector>

#include "an_messages/trajectory.h"

enum CellType
{
	OPEN,
	CLOSED,
	DEFAULT
};

struct Possible_Ending_State_s {
	int ending_state_id;
	double probability;
};

struct Successor_s {
	int action_id;
	double action_cost;
	std::vector<Possible_Ending_State_s> possible_ending_states;
};

struct TrajPt_s {
	double x;
	double y;
	double theta;
	double secFromStart;
};

struct Traj_s {
	std::vector<TrajPt_s> traj;
	double probability;
};

struct Obstacle_s {
	double length;
	double width;
	std::vector<Traj_s> trajs;
};

struct Point_s {
	double x;
	double y;
};

struct Line_s {
	Point_s a;
	Point_s b;
};

struct Observation_s {
	double change;
	double x;
	double y;
	double t;
};

struct MotionPrim_s {
	double length;
	double duration;
	double cost;
	an_messages::trajectory traj;
	int ending_state_id_offset;
};

struct Node_s {
	int id = -1;
	double x = -1;
	CellType type = DEFAULT;
	std::vector<Successor_s> successors;
	int best_action = -1;
	double v = 10000;
};

struct Coordinates_s {
	int m;
	int n;
	int T;
	int intent;
};

#endif