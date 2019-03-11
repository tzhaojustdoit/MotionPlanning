#include "MockPerception.h"

MockPerception::MockPerception(std::vector<char> actual_map, int row, int col) : actual_map_(actual_map), row_(row), col_(col)
{
}

void MockPerception::PerceiveSurroundings(std::vector<bool>& map, int location)
{
	// perceive 4 adjacent cells in this order: right, down, left, up
	if ((location + 1) % col_ != 0) {  // boundary check (right boundary)
		PerceiveLocation(map, location + 1);
	}
	if (location < (row_ - 1) * col_) {  // boundary check (down boundary)
		PerceiveLocation(map, location + col_);
	}
	if (location % col_ != 0) {  // boundary check (left boundary)
		PerceiveLocation(map, location - 1);
	}
	if (location >= col_) {  // boundary check (up boundary)
		PerceiveLocation(map, location - col_);
	}
}

int MockPerception::Localize()
{
	for(unsigned i = 0; i < actual_map_.size(); i++) {
		if (actual_map_[i] == 's') {
			return i;
		}
	}
	return -1;
}

void MockPerception::PerceiveLocation(std::vector<bool> & obstacles, int location)
{
	if (actual_map_[location] == 'x') {
		obstacles[location] = true;  // update map, mark the location as blocked
	}
}
