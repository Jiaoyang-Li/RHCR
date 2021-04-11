#include "KivaGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>

bool KivaGrid::load_map(std::string fname)
{
    std::size_t pos = fname.rfind('.');      // position of the file extension
    auto ext_name = fname.substr(pos, fname.size());     // get the name without extension
    if (ext_name == ".grid")
        return load_weighted_map(fname);
    else if (ext_name == ".map")
        return load_unweighted_map(fname);
    else
    {
        std::cout << "Map file name should end with either .grid or .map. " << std::endl;
        return false;
    }
}

bool KivaGrid::load_weighted_map(std::string fname)
{
	std::string line;
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
	getline(myfile, line); // skip the words "grid size"
	getline(myfile, line);
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	this->rows = atoi((*beg).c_str()); // read number of cols
	beg++;
	this->cols = atoi((*beg).c_str()); // read number of rows
	move[0] = 1;
	move[1] = -cols;
	move[2] = -1;
	move[3] = cols;

	getline(myfile, line); // skip the headers

	//read tyeps and edge weights
	this->types.resize(rows * cols);
	this->weights.resize(rows * cols);
	for (int i = 0; i < rows * cols; i++)
	{
		getline(myfile, line);
		boost::tokenizer< boost::char_separator<char> > tok(line, sep);
		beg = tok.begin();
		beg++; // skip id
		this->types[i] = std::string(beg->c_str()); // read type
		beg++;
		if (types[i] == "Home")
			this->agent_home_locations.push_back(i);
		else if (types[i] == "Endpoint")
			this->endpoints.push_back(i);
		beg++; // skip x
		beg++; // skip y
		weights[i].resize(5);
		for (int j = 0; j < 5; j++) // read edge weights
		{
			if (std::string(beg->c_str()) == "inf")
				weights[i][j] = WEIGHT_MAX;
			else
				weights[i][j] = std::stod(beg->c_str());
			beg++;
		}
	}

	myfile.close();
	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Map size: " << rows << "x" << cols << " with ";
	cout << endpoints.size() << " endpoints and " <<
		agent_home_locations.size() << " home stations." << std::endl;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
	return true;
}


// load map
bool KivaGrid::load_unweighted_map(std::string fname)
{
    std::string line;
    std::ifstream myfile ((fname).c_str());
	if (!myfile.is_open())
    {
	    std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }
	
    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
    map_name = fname.substr(0, pos);     // get the name without extension
    getline (myfile, line); 
	
	
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	rows = atoi((*beg).c_str()); // read number of rows
	beg++;
	cols = atoi((*beg).c_str()); // read number of cols
	move[0] = 1;
	move[1] = -cols;
	move[2] = -1;
	move[3] = cols;

	std::stringstream ss;
	getline(myfile, line);
	ss << line;
	int num_endpoints;
	ss >> num_endpoints;

	int agent_num;
	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> agent_num;

	ss.clear();
	getline(myfile, line);
	ss << line;
	int maxtime;
	ss >> maxtime;

	//this->agents.resize(agent_num);
	//endpoints.resize(num_endpoints + agent_num);
	types.resize(rows * cols);
	weights.resize(rows*cols);
	//DeliverGoal.resize(row*col, false);
	// read map
	//int ep = 0, ag = 0;
	for (int i = 0; i < rows; i++)
	{
		getline(myfile, line);
		for (int j = 0; j < cols; j++)
		{
			int id = cols * i + j;
			weights[id].resize(5, WEIGHT_MAX);
			if (line[j] == '@') // obstacle
			{
				types[id] = "Obstacle";
			}
			else if (line[j] == 'e') //endpoint
			{
				types[id] = "Endpoint";
				weights[id][4] = 1;
				endpoints.push_back(id);
			}
			else if (line[j] == 'r') //robot rest
			{
				types[id] = "Home";
				weights[id][4] = 1;
				agent_home_locations.push_back(id);
			}
			else
			{
				types[id] = "Travel";
				weights[id][4] = 1;
			}
		}
	}
	shuffle(agent_home_locations.begin(), agent_home_locations.end(), std::default_random_engine());
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
			else
				weights[i][dir] = WEIGHT_MAX;
		}
	}
	

	myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with ";
	cout << endpoints.size() << " endpoints and " <<
	agent_home_locations.size() << " home stations." << std::endl;		
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

void KivaGrid::preprocessing(bool consider_rotation)
{
	std::cout << "*** PreProcessing map ***" << std::endl;
	clock_t t = std::clock();
	this->consider_rotation = consider_rotation;
	std::string fname;
	if (consider_rotation)
		fname = map_name + "_rotation_heuristics_table.txt";
	else
		fname = map_name + "_heuristics_table.txt";
	std::ifstream myfile(fname.c_str());
	bool succ = false;
	if (myfile.is_open())
	{
		succ = load_heuristics_table(myfile);
		myfile.close();
	}
	if (!succ)
	{
		for (auto endpoint : endpoints)
		{
			heuristics[endpoint] = compute_heuristics(endpoint);
		}
		for (auto home : agent_home_locations)
		{
			heuristics[home] = compute_heuristics(home);
		}
		save_heuristics_table(fname);
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}
