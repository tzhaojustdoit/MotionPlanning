#include "Display.h"

void Display::DisplayMap(int row, int col, const std::vector<bool> &obstacles, const std::vector<int> &path, int start, int goal)
{
	std::string result;
	int counter = 0;
	// mark obstacles
	for (unsigned i = 0; i < obstacles.size(); i++)
	{
		if (counter == col)
		{
			result += "\n";
			counter = 0;
		}
		if (obstacles[i])
		{
			result += "x ";
		}
		else if (i == start)
		{
			result += "s ";
		}
		else if (i == goal)
		{
			result += "g ";
		}
		else
		{
			result += "_ ";
		}
		counter++;
	}
	for (unsigned i = 1; i < path.size(); i++)
	{
		int val = path[i];
		result[val * 2 + val / col] = 'o';
	}
	std::cout << result << std::endl;
}
