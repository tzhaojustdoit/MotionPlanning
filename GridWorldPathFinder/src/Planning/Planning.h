/**
 * @file Planning.h
 * @brief Defines the Planning interface.
 * @authur: Tianhua Zhao
 */

#ifndef PLANNING_
#define PLANNING_

#include <vector>

/**
  * @class Planning
  *
  * @brief Interface of planning module
  */
class Planning
{
  public:
	/**
	 *@brief ctor
	 */
	Planning(){};

	/**
	 *@brief dtor
	 */
	virtual ~Planning(){};

	/**
	 *@brief set goal location
	 *@param goal goal location
	 */
	virtual void SetGoal(int goal) = 0;

	/**
	 *@brief find the shortest path.
	 *@param obstacles: contains obstacle info
	 *@param location: starting location
	 */
	virtual std::vector<int> FindPath(const std::vector<bool> &map, int location) = 0;

	/**
	 *@brief get number of searches
	 */
	virtual int GetNumOfSearches() const = 0;

	/**
	 *@brief get number of nodes expanded
	 */
	virtual int GetNumOfNodesExpanded() const = 0;
};
#endif // !PLANNING_
