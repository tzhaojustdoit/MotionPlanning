/**
 * @file AutonomousNavigation.h
 * @brief Defines the AutonomousNavigation class.
 * @authur: Tianhua Zhao
 */

#ifndef AUTONOMOUS_NAVIGATION_
#define AUTONOMOUS_NAVIGATION_

#include <vector>
#include <iostream>

#include "../Perception/MockPerception.h"
#include "../Planning/Planning.h"

/**
 * @class AutonomousNavigation
 *
 * @brief an autonomous navigating system. The vehicle assumes the world is empty initially, and gradually
 * gains knowledge about the world as exploring to the goal
 */
class AutonomousNavigation
{
public:
	/**
	 *@brief ctor
	 *@param rows, cols :discritized world frame, number of rows and columns in the grid world.
	 *@param perception_unit: the mock perception unit
	 *@param planning_unit: the planning unit
	 */
	AutonomousNavigation(int rows, int cols, MockPerception* perception_unit, Planning* planning_unit);

	/**
	 *@brief dtor
	 */
	~AutonomousNavigation();

	/**
	 * @brief Set destination. Do initial planning.
	 * @param goal: goal location
	 */
	void SetDestination(int goal);

	/**
	 * @brief Start navigating according to the planned path, perceive surroundings as the vehicle moves.
	 * if the planned path is blocked, replan, navigate... repeat until the vehicle reaches the goal or is unable to find a path.
	 */
	void AutoNavigate();

	/**
	 * @brief number of searches by the planning module
	 */
	int GetNumOfSearches();

	/**
	 * @brief number of nodes expanded by the planning module
	 */
	int GetNumOfNodesExpanded();

private:
	MockPerception *perception_unit_;
	Planning *planning_unit_;
	int rows_;
	int cols_;
	std::vector<int> path_;
	std::vector<bool> obstacles_;
	int current_location_;
	int goal_location_;

	/**
	 *@brief localize the vehicle, perceive surroundings.
	 */
	void Initialize();

	/**
	 *@brief navigate towards the destination according to the planned path.
	 *@return true if reached the destination, false if blocked by an obstacle.
	 */
	bool Navigate();
};
#endif // !AUTONOMOUS_NAVIGATION_

