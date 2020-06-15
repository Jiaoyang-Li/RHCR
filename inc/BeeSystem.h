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
	int get_num_of_missed_tasks() const; // the robots reach these tasks after their deadlines
	int get_num_of_remaining_tasks() const; // the robots have not complete these tasks
	list<int> get_missed_flower_ids() const; // return the ids of the flowers that the robot supposed to complete but didn't
	int get_makespan();
	int get_flowtime() const;
	int get_flowtime_lowerbound() const;
	int get_objective() const;
	double loading_time = 0; // time for loading tasks from file, in seconds
	double VRP_time = 0; // time for solving the VRP problem
	void simulate();
private:
	const BeeGraph& G;
	vector<list<pair<int, int> > > task_sequences; // one task sequence per agent
	void initialize();
	void initialize_start_locations();
	void update_goal_locations();
	void solve_VRP_by_LKH3(string fname);
};

