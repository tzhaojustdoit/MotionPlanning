#include "AutonomousNavigation.h"

AutonomousNavigation::AutonomousNavigation(int rows, int cols, MockPerception * percepttion_unit, Planning * planning_unit) : rows_(rows), cols_(cols), perception_unit_(percepttion_unit), planning_unit_(planning_unit)
{
	Initialize();
}

AutonomousNavigation::~AutonomousNavigation()
{
}

void AutonomousNavigation::Initialize()
{
	obstacles_.resize(rows_ * cols_);
	current_location_ = perception_unit_->Localize();
	perception_unit_->PerceiveSurroundings(obstacles_, current_location_);
}

void AutonomousNavigation::SetDestination(int goal)
{
	goal_location_ = goal;
	planning_unit_->SetGoal(goal);

	//initial planning
	path_ = planning_unit_->FindPath(obstacles_, current_location_);
}

void AutonomousNavigation::AutoNavigate()
{
	while (!path_.empty() && !Navigate()) {
		path_ = planning_unit_->FindPath(obstacles_, current_location_);
	}
	// print number of nodes expanded
	std::cout << std::endl << "[navigation] Auto-navigation ended." << std::endl << "[navigation] Total number of nodes expanded is: " << GetNumOfNodesExpanded() << "." << std::endl;
}

int AutonomousNavigation::GetNumOfSearches()
{
	return planning_unit_->GetNumOfSearches();
}

int AutonomousNavigation::GetNumOfNodesExpanded()
{
	return planning_unit_->GetNumOfNodesExpanded();
}

bool AutonomousNavigation::Navigate()
{
	std::cout << std::endl << "[navigation] Navigation started." << std::endl;
	// move the vehicle from the start cell to the next cell
	// since start cell is already sensed, it is safe to move one step
	current_location_ = path_.back();
	// move the vehicle along the shortest path until it is blocked by an obstacle or reaches the goal
	int counter = path_.size() - 1;
	while (counter > 0)
	{
		// sense adjacent cells
		perception_unit_->PerceiveSurroundings(obstacles_, current_location_);
		// get the next path point
		int next = path_[counter - 1];
		// if next cell is blocked, break out of the loop
		if (obstacles_[next])
		{
			std::cout << "[navigation] Obstacle detected, start replanning..." << std::endl;

			return false;
		}
		// move to the next cell
		current_location_ = next;
		counter--;
	}
	std::cout << std::endl << "[navigation] Arrived at the destination." << std::endl;
	return true;
}
