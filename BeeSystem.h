#pragma once
#include "BasicSystem.h"
#include "BeeGraph.h"

class BeeSystem :
	public BasicSystem
{
public:
	BeeSystem(const BeeGraph& G, MAPFSolver& solver);
	~BeeSystem();
	bool load_task_assignments(string fname);
	int get_num_of_remaining_tasks() const;
	int get_makespan();
	int get_flowtime() const;
	int get_flowtime_lowerbound() const;
	double loading_time = 0; // time for loading tasks from file, in seconds
	void simulate();
private:
	const BeeGraph& G;
	vector<list<int>> task_sequences; // one task sequence per agent
	int entrance; // the collisions at the entrance will be ignored.
	void initialize();
	void initialize_start_locations();
	void update_goal_locations();
};

