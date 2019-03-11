/**
 * @file Display.h
 * @brief Defines functions related to display.
 * @authur: Tianhua Zhao
 */

#ifndef DISPLAY_
#define DISPLAY_

#include <vector>
#include <iostream>
#include <string>

#include "../Planning/library/Node.h"

namespace Display
{

/**
	 * @brief display the planned navigation map.
	 * @param row, col number of rows and columns of the map
	 * @param map contains obsacle info of the map
	 * @param path contains path info of the map
	 * @param start, goal contains start adn goal location of the map
	 */
void DisplayMap(int row, int col, const std::vector<bool> &obstacles, const std::vector<int> &path, int start, int goal);
} // namespace Display
#endif // !DISPLAY_
