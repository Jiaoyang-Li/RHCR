#include "SortingGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>

bool SortingGrid::load_map(std::string fname)
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
    getline (myfile, line); // skip the words "grid size"
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

	//read tyeps, station ids and edge weights
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
		if (types[i] == "Induct")
			this->inducts[beg->c_str()] = i; // read induct station id
		else if (types[i] == "Eject")
		{
			boost::unordered_map<std::string, std::list<int> >::iterator it = ejects.find(beg->c_str());
			if (it == ejects.end())
			{
				this->ejects[beg->c_str()] = std::list<int>();
			}
			this->ejects[beg->c_str()].push_back(i); // read eject station id
		}
		beg++;
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
    std::cout << "Map size: " << rows << "x" << cols << " with " << inducts.size() << " induct stations and " <<
        ejects.size() << " eject stations." << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}


void SortingGrid::preprocessing(bool consider_rotation)
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
		// ensure that the heuristic table is correct
		for (auto h : heuristics)
		{
			if (types[h.first] != "Induct" && types[h.first] != "Eject")
			{
				cout << "The heuristic table does not match the map!" << endl;
				exit(-1);
			}
		}
	}
	if (!succ)
	{
		for (auto induct : inducts)
		{
			heuristics[induct.second] = compute_heuristics(induct.second);
		}
		for (auto eject_station : ejects)
		{
			for (int eject : eject_station.second)
			{
				heuristics[eject] = compute_heuristics(eject);
			}
		}
		save_heuristics_table(fname);
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}
