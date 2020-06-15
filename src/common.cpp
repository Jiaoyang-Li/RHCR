#include "common.h"


bool validMove(int curr, int next, int map_size, int num_col)
{
	if (next < 0 || next >= map_size)
		return false;
	int curr_x = curr / num_col;
	int curr_y = curr % num_col;
	int next_x = next / num_col;
	int next_y = next % num_col;
	return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint) << "," <<
		std::get<2>(constraint) << "," << std::get<3>(constraint) << "," <<
		std::get<4>(constraint) << ">";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	os << "<" << std::get<0>(conflict) << "," << std::get<1>(conflict) << "," <<
		std::get<2>(conflict) << "," << std::get<3>(conflict) << "," <<
		std::get<4>(conflict) << ">";
	return os;
}

ostream& operator<<(ostream& os, const Interval& interval)
{
    os << "[" << std::get<0>(interval) << "," << std::get<1>(interval) << ")(" <<
       std::get<2>(interval) << ")";
    return os;
}