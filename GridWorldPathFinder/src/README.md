Using A* and Adaptive A* algorithms to navigate an agent in a GridWorld.

Under /src directory:

Compile with:
g++ Drivers/Driver.cpp AutoNav/AutonomousNavigation.cpp AutoNav/AutonomousNavigation.h Perception/MockPerception.cpp Perception/MockPerception.h Planning/AdaptiveAStar/AdaptiveAStarPlanning.cpp Planning/AdaptiveAStar/AdaptiveAStarPlanning.h Planning/AStar/AStarPlanning.cpp Planning/AStar/AStarPlanning.h Planning/library/CellType.h Planning/library/Node.cpp Planning/library/Node.h Planning/library/OpenList.cpp Planning/library/OpenList.h Planning/Planning.h Utilities/Display.cpp Utilities/Display.h Utilities/Read.cpp Utilities/Read.h -std=c++11 -o Astar

Run with the following flags: Astar -i inputfile [-A]
-i is followed by input file name
Use -A if running with Adaptive A*

An example command line input for A* would be:
./Astar -i ../maps/5.txt

An example command line input for Adaptive A* would be:
./Astar -i ../maps/5.txt -A
