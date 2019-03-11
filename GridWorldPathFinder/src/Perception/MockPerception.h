/**
 * @file MockPerception.h
 * @brief Defines the MockPerception class.
 * @authur: Tianhua Zhao
 */

 /**
  * @file MockPerception.h
  * @brief Defines the MockPerception class.
  * @authur: Tianhua Zhao
  */

#ifndef MOCK_PERCEPTION_
#define MOCK_PERCEPTION_

#include <vector>

#include "../Planning/library/Node.h"

 /**
  * @class MockPerception
  *
  * @brief Mock perception class by reading data from an char array.
  */
class MockPerception
{
public:
	/**
	 * @brief construct a mock perception module with given actual map,
	 *        simluate sensing process by checking the actual map.
	 * @param map the actual world map
	 * @param row, col, number of rows and colums in the grid world map
	 */
	MockPerception(std::vector<char> actual_map, int row, int col);

	/**
	 * @brief perceive the surroundings of a location, update obstacle info to the map
	 * @param map contains obsacle info of the map
	 * @param location the location to perceive surroudings
	 */
	void PerceiveSurroundings(std::vector<bool> & obstacles, int location);

	/**
	 * @brief return the vehicle's current location on the map
	 */
	int Localize();
private: 
	std::vector<char> actual_map_; // the map of the actual world, i.e., the map to explore.
	int row_;  // number of rows
	int col_;  // number of columns

	/**
	 * @brief perceive the location, update obstacle info to the map
	 * @param map contains obsacle info of the map
	 * @param location the location to perceive
	 */
	void PerceiveLocation(std::vector<bool> &map, int location);
};
#endif // !MOCK_PERCEPTION_