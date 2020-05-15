#include "SortingSystem.h"
#include <stdlib.h>
#include "PBS.h"
#include <boost/tokenizer.hpp>
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"


SortingSystem::SortingSystem(const SortingGrid& G, MAPFSolver& solver): BasicSystem(G, solver), c(8), G(G) {}


SortingSystem::~SortingSystem() {}


void SortingSystem::initialize_start_locations()
{
    int N = G.size();
    std::vector<bool> used(N, false);

    // Choose random start locations
    // Any non-obstacle locations can be start locations
    // Start locations should be unique
	for (int k = 0; k < num_of_drives;)
	{
		int loc = rand() % N;
		if (G.types[loc] != "Obstacle" && !used[loc])
		{
			int orientation = -1;
			if (consider_rotation)
			{
				orientation = rand() % 4;
			}
			starts[k] = State(loc, 0, orientation);
			paths[k].emplace_back(starts[k]);
			used[loc] = true;
			finished_tasks[k].emplace_back(loc, 0);
			k++;
		}
	}
}


void SortingSystem::initialize_goal_locations()
{
	if (hold_endpoints || useDummyPaths)
		return;
    // Choose random goal locations
    // a close induct location can be a goal location, or
    // any eject locations can be goal locations
    // Goal locations are not necessarily unique
    for (int k = 0; k < num_of_drives; k++)
    {
		int goal;
		if (k % 2 == 0) // to induction
		{
			goal = assign_induct_station(starts[k].location);
			drives_in_induct_stations[goal]++;
		}
		else // to ejection
		{
			goal = assign_eject_station();
		}
		goal_locations[k].emplace_back(goal);
    }
}


void SortingSystem::update_goal_locations()
{
	for (int k = 0; k < num_of_drives; k++)
	{
		int curr = paths[k][timestep].location; // current location

		int goal; // The last goal location
		if (goal_locations[k].empty())
		{
			goal = curr;
		}
		else
		{
			goal = goal_locations[k].back();
		}
		double min_timesteps = G.get_Manhattan_distance(curr, goal); // cannot use h values, because graph edges may have weights  // G.heuristics.at(goal)[curr];
		while (min_timesteps <= simulation_window)
			// The agent might finish its tasks during the next planning horizon
		{
			// assign a new task
			int next;
			if (G.types[goal] == "Induct")
			{
				next = assign_eject_station();
			}
			else if (G.types[goal] == "Eject")
			{
				next = assign_induct_station(curr);
				drives_in_induct_stations[next]++; // the drive will go to the next induct station
			}
			else
			{
				std::cout << "ERROR in update_goal_function()" << std::endl;
				std::cout << "The fiducial type should not be " << G.types[curr] << std::endl;
				exit(-1);
			}
			goal_locations[k].emplace_back(next);
			min_timesteps += G.get_Manhattan_distance(next, goal); // G.heuristics.at(next)[goal];
			goal = next;
		}
	}
}


int SortingSystem::assign_induct_station(int curr) const
{
    int assigned_loc;
	double min_cost = DBL_MAX;
	for (auto induct : drives_in_induct_stations)
	{
		double cost = G.heuristics.at(induct.first)[curr] + c * induct.second;
		if (cost < min_cost)
		{
			min_cost = cost;
			assigned_loc = induct.first;
		}
	}
    return assigned_loc;
}


int SortingSystem::assign_eject_station() const
{
	int n = rand() % G.ejects.size();
	boost::unordered_map<std::string, std::list<int> >::const_iterator it = G.ejects.begin();
	std::advance(it, n);
	int p = rand() % it->second.size();
	auto it2 = it->second.begin();
	std::advance(it2, p);
	return *it2;
}

void SortingSystem::simulate(int simulation_time)
{
    std::cout << "*** Simulating " << seed << " ***" << std::endl;
    this->simulation_time = simulation_time;
    initialize();
	
	for (; timestep < simulation_time; timestep += simulation_window)
	{
		std::cout << "Timestep " << timestep << std::endl;

		update_start_locations();
		update_goal_locations();
		solve();

		// move drives
		auto new_finished_tasks = move();
		std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

		// update tasks
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			finished_tasks[id].emplace_back(loc, t);
			num_of_tasks++;
			if (G.types[loc] == "Induct")
			{
				drives_in_induct_stations[loc]--; // the drive will leave the current induct station
			}
		}
		
		

		if (congested())
		{
			cout << "***** Too many traffic jams ***" << endl;
			break;
		}
	}

    update_start_locations();
    std::cout << std::endl << "Done!" << std::endl;
    save_results();
}


void SortingSystem::initialize()
{
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);

	for (const auto induct : G.inducts)
	{
		drives_in_induct_stations[induct.second] = 0;
	}

	bool succ = load_records(); // continue simulating from the records
	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
			cout << "Randomly generating initial locations" << endl;
			initialize_start_locations();
			initialize_goal_locations();
		}
	}

	// initialize induct station counter
	for (int k = 0; k < num_of_drives; k++)
	{
		// goals
		int goal = goal_locations[k].back();
		if (G.types[goal] == "Induct")
		{
			drives_in_induct_stations[goal]++;
		}
		else if (G.types[goal] != "Eject")
		{
			std::cout << "ERROR in the type of goal locations" << std::endl;
			std::cout << "The fiducial type of the goal of agent " << k << " is " << G.types[goal] << std::endl;
			exit(-1);
		}
	}
}
