#pragma once
#include "BasicSystem.h"
#include "OnlineGraph.h"

class OnlineSystem: public BasicSystem
{
public:
	OnlineSystem(const OnlineGrid& G, MAPFSolver& solver);
	~OnlineSystem();

	void simulate(int simulation_time);


private:
	const OnlineGrid& G;
	double lambda; // param for Possion distribution
	list<Path> finished_paths; // store the paths of agents that have reached their goal locations
	void move(); // overwrite
	void save_results(); // overwrite
	void update_start_and_goal_locations(int num_of_new_agents);

};

