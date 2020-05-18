#include "OnlineSystem.h"
#include <random>


OnlineSystem::OnlineSystem(const OnlineGrid& G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {}


OnlineSystem::~OnlineSystem()
{
}


// TODO: load records for online system


void OnlineSystem::update_start_and_goal_locations(int num_of_new_agents)
{
    // update the start states of the existing agents. Their goal locations remain unchanged.
	starts.clear();
	list<int> available_entries;
	for (const auto& entry : G.entries)
		available_entries.push_back(entry);

	for (const auto& path : paths)
	{
		int loc = path[timestep].location;
		starts.emplace_back(loc, 0, path[timestep].orientation);
		if (G.types[loc] == "Entry")
		{
			available_entries.remove(loc);
		}
	}

	// add start states and goal locations for new agents
	int count = 0;
	while (count < num_of_new_agents && !available_entries.empty())
	{
		// randomly select an available entry cell as the start
		int idx = rand() % available_entries.size();
		auto pt = available_entries.begin();
		std::advance(pt, idx);
		starts.emplace_back(*pt, 0, -1);
		paths.emplace_back(timestep); // insert a new path of length=timestep for the new agent
		paths.back().emplace_back(*pt, 0, -1);		
		available_entries.erase(pt); // set this cell to unavailable
		 // randomly select ab exit cell as the goal
		idx = rand() % G.exits.size();
		goal_locations.emplace_back();
		goal_locations.back().emplace_back(G.exits[idx], 0);
		count++;
	}
	num_of_drives = (int)starts.size();
	cout << count << " new agents arrive and there are " << num_of_drives << " agents in total." << endl;
	if (available_entries.empty())
		cout << "The entry locations are fully occupied!" << endl;

}

void OnlineSystem::move()
{
	int start_timestep = timestep;
	int end_timestep = timestep + simulation_window;

	for (int t = start_timestep; t <= end_timestep; t++)
	{
		auto path = paths.begin();
		auto goal = goal_locations.begin();
		while (path != paths.end())
		{
			State curr = path->at(t);
			// check whether the move is valid
			if (t > 0 && path->at(t - 1).timestep >= 0)
			{
				State prev = path->at(t - 1);
				if (curr.location == prev.location)
				{
					if (G.get_rotate_degree(prev.orientation, curr.orientation) == 2)
					{
						cout << "A drive rotates 180 degrees from " << prev << " to " << curr << endl;
						save_results();
						exit(-1);
					}
				}
				else if (consider_rotation)
				{
					if (prev.orientation != curr.orientation)
					{
						cout << "A drive rotates while moving from " << prev << " to " << curr << endl;
						save_results();
						exit(-1);
					}
					else if (!G.valid_move(prev.location, prev.orientation) ||
						prev.location + G.move[prev.orientation] != curr.location)
					{
						cout << "A drive jumps from " << prev << " to " << curr << endl;
						save_results();
						exit(-1);
					}
				}
				else
				{
				int dir = G.get_direction(prev.location, curr.location);
				if (dir < 0 || !G.valid_move(prev.location, dir))
				{
				cout << "A drive jumps from " << prev << " to " << curr << endl;
				save_results();
				exit(-1);
				}
				}
			}

			// Check whether this move has conflicts with other agents
			auto other_path = path;
			++other_path;
			for (;other_path != paths.end(); ++other_path)
			{
				for (int i = max(0, t - k_robust); i <= min(t + k_robust, end_timestep); i++)
				{
					if ((int)other_path->size() <= i)
						break;
					if (other_path->at(i).location == curr.location)
					{
						cout << "Two drives have a conflict at location " << curr.location << " at timestep " << i << endl;
						save_results(); //TODO: write termination reason to files
						exit(-1);
					}
				}
			}

			// Check whether the agent has reached its goal location
			if ((int)path->size() == t + 1 && path->back().location == goal->at(0).first)
			{
				auto start = path->begin(); // find the start state of the agent
				while (start->timestep < 0)
				{
					++start;
				}
				finished_paths.emplace_back(); // insert the path to finsihed paths
				finished_paths.back().insert(finished_paths.back().end(), start, path->end());
				path = paths.erase(path); // remove the path from the current path set
				goal = goal_locations.erase(goal);
			}
			else
			{
				++path; // move to the next path
				++goal;
			}
		}
	}
	return;
}


void OnlineSystem::simulate(int simulation_time)
{
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	this->simulation_time = simulation_time;
	initialize_solvers();
	lambda = num_of_drives;
	std::default_random_engine generator;
	std::poisson_distribution<int> distribution(lambda);

	for (; timestep < simulation_time; timestep += simulation_window)
	{
		std::cout << "Timestep " << timestep << std::endl;

		int num_of_new_agents = distribution(generator);
		update_start_and_goal_locations(num_of_new_agents);

		solve();

		// move drives
		size_t old = paths.size();
		move();
		std::cout << old - paths.size() << " agents have reached their goal locations" << std::endl;

		if (congested())
		{
			cout << "***** Too many traffic jams ***" << endl;
			break;
		}
	}

	std::cout << std::endl << "Done!" << std::endl;
	save_results();
}


void OnlineSystem::save_results()
{
	std::cout << "*** Saving " << seed << " ***" << std::endl;
	clock_t t = std::clock();
	std::ofstream output;

	// settings
	output.open(outfile + "/config.txt", std::ios::out);
	output << "map: " << G.map_name << std::endl
		<< "#drives: " << num_of_drives << std::endl
		<< "seed: " << seed << std::endl
		<< "solver: " << solver.get_name() << std::endl
		<< "time_limit: " << time_limit << std::endl
		<< "simulation_window: " << simulation_window << std::endl
		<< "planning_window: " << planning_window << std::endl
		<< "simulation_time: " << simulation_time << std::endl
		<< "robust: " << k_robust << std::endl
		<< "rotate: " << consider_rotation << std::endl
		<< "use_dummy_paths: " << useDummyPaths << std::endl
		<< "hold_endpoints: " << hold_endpoints << std::endl;

	output.close();


	// paths
	output.open(outfile + "/paths.txt", std::ios::out);
	output << finished_paths.size() << std::endl;
	for (const auto& path : finished_paths)
	{
		for (auto p : path)
		{
			if (p.timestep <= timestep)
				output << p << ";";
		}
		output << std::endl;
	}
	output.close();
	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}