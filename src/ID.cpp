#include"ID.h"


bool ID::run(const vector<State>& starts,
	const vector< vector<pair<int, int> > >& goal_locations,
	int time_limit)
{
    this->starts = starts;
	this->goal_locations = goal_locations;
	this->time_limit = time_limit;
	this->num_of_agents = (int)starts.size();
	this->solution.resize(num_of_agents);
	this->group_ids.resize(num_of_agents);

	for (int i = 0; i < num_of_agents; i++)
		group_ids[i] = i;

	solution_found = true;
	for (int i = 0; i < num_of_agents; i++)
	{
		if (!plan_paths_for_group(i))
		{
			solution_found = false;
		}
	}	

	this->runtime = (clock() - start_time) * 1.0 / CLOCKS_PER_SEC;
	if (screen > 0) // 1 or 2
		print_results();
	return true;
}


bool ID::plan_paths_for_group(int group_id)
{
	vector<State> curr_starts;
	vector< vector<pair<int, int> > > curr_goal_locations;
	vector<int> curr_agents;
	for (int i = 0; i < num_of_agents; i++)
	{
		if (group_ids[i] == group_id)
		{
			curr_starts.push_back(starts[i]);
			curr_goal_locations.push_back(goal_locations[i]);
			curr_agents.push_back(i);
			// solver.initial_paths.push_back(solution[i]); // TODO: can we generate initial paths for ID?
		}
		else if (!solution[i].empty())
		{
			solver.initial_soft_path_constraints.push_back(&solution[i]);
		}
	}
	this->runtime = (clock() - start_time) * 1.0 / CLOCKS_PER_SEC;
	bool sol = solver.run(curr_starts, curr_goal_locations, time_limit - this->runtime);
	if (!sol)
		return false;

	// update paths
	for (int i = 0; i < (int)curr_agents.size(); i++)
	{
		solution[curr_agents[i]] = solver.solution[i];
	}
	solver.clear();

	// check conflicts and merge agents if necessary
	for (int i = 0; i < (int) curr_agents.size(); i++)
	{
		const Path& path1 = solver.solution[i];
		for (int j = 0; j < num_of_agents; j++)
		{
			if (group_ids[j] == group_id)
				continue;
			const Path& path2 = solution[j];
			if (has_conflicts(path1, path2)) // merge the group
			{
				for (auto id : curr_agents)
				{
					group_ids[id] = group_ids[j];
				}
				if (plan_paths_for_group(group_ids[j]))
					return true;
				else
					return false;
			}
		}
	}
	return true;
}


bool ID::has_conflicts(const Path& path1, const Path& path2) const
{

	// TODO: add k-robust
	if (solver.hold_endpoints)
	{
		size_t min_path_length = path1.size() < path2.size() ? path1.size() : path2.size();
		for (size_t timestep = 0; timestep < min_path_length; timestep++)
		{
			int loc1 = path1[timestep].location;
			int loc2 = path2[timestep].location;
			if (loc1 == loc2)
			{
				return true;
			}
			else if (timestep < min_path_length - 1
				&& loc1 == path2[timestep + 1].location
				&& loc2 == path1[timestep + 1].location)
			{
				return true;
			}
		}

		if (path1.size() < path2.size())
		{
			int loc1 = path1.back().location;
			for (size_t timestep = min_path_length; timestep < path2.size(); timestep++)
			{
				int loc2 = path2[timestep].location;
				if (loc1 == loc2)
				{
					return true;
				}
			}
		}
		else if (path2.size() < path1.size())
		{
			int loc2 = path2.back().location;
			for (size_t timestep = min_path_length; timestep < path1.size(); timestep++)
			{
				int loc1 = path1[timestep].location;
				if (loc1 == loc2)
				{
					return true;
				}
			}
		}
	}
	else
	{
		int size1 = min(solver.window + 1, (int)path1.size());
		int size2 = min(solver.window + 1, (int)path2.size());
		for (int timestep = 0; timestep < size1; timestep++)
		{
			if (size2 <= timestep - k_robust)
				break;
			else if (k_robust > 0)
			{
				int loc = path1[timestep].location;
				for (int i = max(0, timestep - k_robust); i <= min(timestep + k_robust, size2 - 1); i++)
				{
					if (loc == path2[i].location)
					{
						return true;
					}
				}
			}
			else
			{

				int loc1 = path1[timestep].location;
				int loc2 = path2[timestep].location;
				if (loc1 == loc2)
				{
					return true;
				}
				else if (timestep < size1 - 1 && timestep < size2 - 1
					&& loc1 == path2[timestep + 1].location
					&& loc2 == path1[timestep + 1].location)
				{
					return true;
				}
			}

		}
	}
	return false;
}


void ID::save_results(const string &fileName, const string &instanceName) const
{
	list<pair<int, int>> groups;
	for (int id : group_ids)
	{
		list<pair<int, int>>::iterator group = groups.begin();
		for (; group != groups.end(); ++group)
		{
			if (group->first == id)
			{
				group->second++;
				break;
			}
		}
		if (group == groups.end())
		{
			groups.emplace_back(id, 1);
		}
	}
	int num_of_groups = (int)groups.size();
	int largest_group = 0;
	for (auto group : groups)
	{
		if (group.second > largest_group)
			largest_group = group.second;
	}

	std::ofstream stats;
	stats.open(fileName, std::ios::app);
	stats << runtime << "," <<
		num_of_groups << "," << largest_group << "," <<
		0 << "," << 0 << "," <<
		0 << "," << 0 << "," <<
		0 << "," << 0 << "," <<
		instanceName << std::endl;
	stats.close();
}

void ID::print_results() const
{
	std::cout << get_name() << ":";
	if (solution_found) // solved
		std::cout << "Succeed,";
	else if (runtime > time_limit) // time_out
		std::cout << "Timeout,";
	else // no solution
		std::cout << "No solutions,";
	int num_of_groups = 0;
	int largest_group = 0;
	if (solution_found)
	{
		list<pair<int, int>> groups;
		for (int id : group_ids)
		{
			list<pair<int, int>>::iterator group = groups.begin();
			for (; group != groups.end(); ++group)
			{
				if (group->first == id)
				{
					group->second++;
					break;
				}
			}
			if (group == groups.end())
			{
				groups.emplace_back(id, 1);
			}
		}
		num_of_groups = (int)groups.size();
		largest_group = 0;
		for (auto group : groups)
		{
			if (group.second > largest_group)
				largest_group = group.second;
		}
	}
	std::cout << runtime << "," <<
		num_of_groups << "," << largest_group << "," <<
		0 << "," << 0 << "," <<
		0 << "," << 0 << "," <<
		0 << "," << 0 << "," <<
		window <<
		std::endl;
}