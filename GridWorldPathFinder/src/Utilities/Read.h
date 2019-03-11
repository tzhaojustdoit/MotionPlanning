/**
 * @file Read.h
 * @brief Defines functions related to reading input.
 * @authur: Tianhua Zhao
 */
#ifndef READ_
#define READ_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

/**
  * @struct MapData
  *
  * @brief map data.
  */
struct MapData
{
	int rows;
	int cols;
	std::vector<char> map;
	int start;
	int goal;
};
namespace Read
{
/**
	 *@brief read map data from a file
	 *@param filename: file name
	 */
MapData ReadMapFile(std::string filename);
} // namespace Read

#endif // !READ_