#include "Read.h"

MapData Read::ReadMapFile(std::string filename)
{
	MapData result;
	std::ifstream input_file_stream(filename.c_str());
	std::string line;
	if (input_file_stream.good())
	{
		input_file_stream >> result.rows;
		input_file_stream >> result.cols;
		result.map.reserve(result.rows * result.cols);
		char cur;
		for (int i = 0; i < result.rows * result.cols; i++)
		{
			input_file_stream >> cur;
			if (cur == 's')
			{
				result.start = i;
			}
			else if (cur == 'g')
			{
				result.goal = i;
			}
			result.map.push_back(cur);
		}
		input_file_stream.close();
	}
	else
	{
		std::cout << "Error openning file." << std::endl;
	}
	return result;
}
