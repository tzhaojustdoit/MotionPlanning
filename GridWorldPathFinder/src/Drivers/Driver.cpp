/**
 * @file Driver.cpp
 * @brief Defines the main.
 * @authur: Tianhua Zhao
 */

#include <vector>
#include <iostream>
#include <string.h>

#include "../AutoNav/AutonomousNavigation.h"
#include "../Planning/Planning.h"
#include "../Planning/AdaptiveAStar/AdaptiveAStarPlanning.h"
#include "../Planning/AStar/AStarPlanning.h"
#include "../Utilities/Read.h"

/*
 *@brief Main driver. 
 */
int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		std::cout << "Invalid argument count." << std::endl;
		return 0;
	}
	std::string filename;
	int counter = 1;
	bool adaptiveAStarFlag = false;
	while (counter < argc){
		if(strcmp(argv[counter], "-i") == 0){
			counter++;
			filename = std::string(argv[counter]);
		}
		else if (strcmp(argv[counter], "-A") == 0){
			adaptiveAStarFlag = true;
		}
		counter++;
	}
	if(filename.empty()){
		std::cout << "File name argument missing.";
		return 0;
	}
	std::cout<< "File = " << filename<<std::endl;
	MapData data = Read::ReadMapFile(filename);
	Planning *planning_unit;
	if (adaptiveAStarFlag)
	{
		planning_unit = new AdaptiveAStarPlanning(data.rows, data.cols);
	}
	else
	{
		planning_unit = new AStarPlanning(data.rows, data.cols);
	}
	MockPerception *perception_unit = new MockPerception(data.map, data.rows, data.cols);
	AutonomousNavigation an(data.rows, data.cols, perception_unit, planning_unit);
	an.SetDestination(data.goal);
	an.AutoNavigate();
	delete planning_unit;
	delete perception_unit;
	return 0;
}