#include "BeeGraph.h"
#include <boost/tokenizer.hpp>
#include <sstream>      // std::istringstream

bool BeeGraph::load_map(string fname)
{
	using namespace boost;
	// read parameters
	std::ifstream myfile((fname).c_str());
	if (!myfile.is_open())
	{
		std::cout << "Parameter file " << fname << " does not exist. " << std::endl;
		return false;
	}
	clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
	map_name = fname.substr(0, pos);     // get the name without extension
	string temp;
	myfile >> temp >> this->rows; // size
	this->cols = this->rows;
	move[0] = 1;
	move[1] = -cols;
	move[2] = -1;
	move[3] = cols;
	int num_of_obstacles;
	myfile >> temp >> num_of_obstacles; // removes
	int num_of_flowers;
	myfile >> temp >> num_of_flowers; // D
	flowers.resize(num_of_flowers);
	flower_demands.resize(num_of_flowers);
	flower_costs.resize(num_of_flowers);
	flower_time_windows.resize(num_of_flowers);
	myfile >> temp >> num_of_bees; // N
	int num_of_initial_locations;
	myfile >> temp >> num_of_initial_locations; // R
	initial_locations.resize(num_of_initial_locations);
	myfile >> temp >> max_timestep; // T
	myfile >> temp >> bee_capacity; // Q
	myfile >> temp; // demand
	for (int i = 0; i < num_of_flowers; i++)
	{
		myfile >> flower_demands[i];
	}
	myfile >> temp >> wait_cost; // theta_1
	myfile >> temp >> move_cost; // theta_2
	myfile >> temp; // theta_d
	for (int i = 0; i < num_of_flowers; i++)
	{
		myfile >> flower_costs[i];
	}
	myfile >> temp; // D_locations
	for (int i = 0; i < num_of_flowers; i++)
	{
		myfile >> flowers[i];
		flowers[i]--;
	}
	myfile >> temp >> this->entrance; // N_location
	this->entrance--;
	myfile >> temp; // R_locations
	for (int i = 0; i < num_of_initial_locations; i++)
	{
		myfile >> initial_locations[i];
		initial_locations[i]--;
	}
	myfile >> temp; // remove_locations
	this->types.resize(rows * cols, "Travel");
	int id;
	for (int i = 0; i < num_of_obstacles; i++)
	{
		myfile >> id;
		types[id - 1] = "Obstacle";
	}
	types[entrance] = "Magic"; // vertex collsions at the Magic vertex are ignored!

	this->weights.resize(rows * cols);
	for (int i = 0; i < cols * rows; i++)
	{
		weights[i].resize(5, WEIGHT_MAX);
		if (types[i] == "Obstacle")
			continue;
		else if (types[i] == "Magic")
			weights[i][4] = wait_cost; //0; // waiting at the entrance has zero costs.
		else
			weights[i][4] = wait_cost; // wait actions are allowed
		for (int dir = 0; dir < 4; dir++)
		{
			if (0 <= i + move[dir] && i + move[dir] < cols * rows && get_Manhattan_distance(i, i + move[dir]) <= 1 && types[i + move[dir]] != "Obstacle")
				weights[i][dir] = move_cost;
		}
	}

	myfile.close();

	// read time windows
	string windowfname;
	pos = map_name.rfind("parameter");
	windowfname = map_name.replace(pos, 9, "D_time_windows");
	windowfname = windowfname + ".csv";
	std::ifstream myfile2(windowfname.c_str());
	if (!myfile2.is_open())
	{
		std::cout << "Time window file " << fname << " does not exist. " << std::endl;
		return false;
	}
	char_separator<char> sep(",");
	for (int i = 0; i < num_of_flowers; i++)
	{
		getline(myfile2, temp);
		tokenizer< char_separator<char> > tok(temp, sep);
		auto beg = tok.begin();
		flower_time_windows[i].first = atoi((*beg).c_str()); // read release time
		beg++;
		flower_time_windows[i].second = atoi((*beg).c_str()); // read deadline
	}
	myfile2.close();
	loading_time = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
	// std::cout << "Map size: " << rows << "x" << cols << std::endl;
	// std::cout << "Done! (" << runtime << " s)" << std::endl;
	return true;
}


bool BeeGraph::load_Nathan_map(string fname)
{
	using namespace boost;
	string line;
	std::ifstream myfile((fname).c_str());
	if (!myfile.is_open())
	{
		std::cout << "Map file " << fname << " does not exist. " << std::endl;
		return false;
	}

	std::cout << "*** Loading map ***" << std::endl;
	clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
	map_name = fname.substr(0, pos);     // get the name without extension

	tokenizer< char_separator<char> >::iterator beg;
	getline(myfile, line); // skip word "type:*"
	char_separator<char> sep(" ");
	getline(myfile, line);
	tokenizer< char_separator<char> > tok(line, sep);
	beg = tok.begin();
	beg++;
	this->rows = atoi((*beg).c_str()); // read number of rows
	getline(myfile, line);
	tokenizer< char_separator<char> > tok2(line, sep);
	beg = tok2.begin();
	beg++;
	this->cols = atoi((*beg).c_str()); // read number of cols
	move[0] = 1;
	move[1] = -cols;
	move[2] = -1;
	move[3] = cols;
	getline(myfile, line); // skip word "map"

	 //read tyeps and edge weights
	this->types.resize(rows * cols);
	this->weights.resize(rows * cols);
	// read map (and start/goal locations)
	for (int i = 0; i < rows; i++) {
		getline(myfile, line);
		for (int j = 0; j < cols; j++) {
			int id = cols * i + j;
			weights[id].resize(5, WEIGHT_MAX);
			if (line[j] == '.')
			{
				this->types[id] = "Travel";
			}
			else
			{
				this->types[id] = "Obstacle";
			}
		}
	}

	for (int i = 0; i < cols * rows; i++)
	{
		if (types[i] == "Obstacle")
		{
			continue;
		}
		for (int dir = 0; dir < 4; dir++)
		{
			if (0 <= i + move[dir] && i + move[dir] < cols * rows && get_Manhattan_distance(i, i + move[dir]) <= 1 && types[i + move[dir]] != "Obstacle")
				weights[i][dir] = 1;
		}
	}
	
	myfile.close();
	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Map size: " << rows << "x" << cols << std::endl;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
	return true;
}


void BeeGraph::preprocessing(string fname, bool consider_rotation)
{
	// std::cout << "*** PreProcessing map ***" << std::endl;
	clock_t t = std::clock();
	this->consider_rotation = consider_rotation;
	heuristics[entrance] = compute_heuristics(entrance);
	std::ifstream myfile((fname).c_str());
	if (myfile.is_open())
	{
		using namespace boost;
		char_separator<char> sep(" ");
		string line;
		int id, i;
		char temp;
		while (getline(myfile, line))
		{
			std::istringstream iss(line);
			iss >> i >> temp; // skip the id of the agent
			while (iss >> id)
			{
				id--;
				if (heuristics.find(flowers[id]) == heuristics.end())
				{
					heuristics[flowers[id]] = compute_heuristics(flowers[id]);
				}
			}
		}
	}
	myfile.close();

	preprocessing_time = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
	// std::cout << heuristics.size() << " flowers!" << std::endl;
	// std::cout << "Done! (" << runtime << " s)" << std::endl;
}
