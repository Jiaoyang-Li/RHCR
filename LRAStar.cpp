#include "LRAStar.h"

LRAStar::LRAStar(const BasicGraph &G, SingleAgentSolver& path_planner): MAPFSolver(G, path_planner), num_expanded(0), num_generated(0) {}


bool LRAStar::run(const vector<State>& starts, const vector< vector<pair<int, int> > >& goal_locations, int time_limit)
{
	clock_t start = std::clock();
	// plan individual paths
	num_of_agents = starts.size();
	num_expanded = 0;
	num_generated = 0;
	vector<Path> shortest_paths(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		runtime = (std::clock() - start) * 1.0 / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return false;
		shortest_paths[i] = find_shortest_path(starts[i], goal_locations[i]);
	}
	// resolve conflicts
	resolve_conflicts(shortest_paths);
	runtime = (std::clock() - start) * 1.0 / CLOCKS_PER_SEC;
	return true;
}


Path LRAStar::find_shortest_path(const State& start, const vector<pair<int, int> >& goal_locations)
{
	// The following is used to generate the hash value of a node
	struct Hasher
	{
		std::size_t operator()(const StateTimeAStarNode* n) const
		{
			size_t loc_hash = std::hash<int>()(n->state.location);
			size_t ori_hash = std::hash<int>()(n->state.orientation);
			return (loc_hash ^ (ori_hash << 1));
		}
	};

	// The following is used to  check whether two nodes are equal
	struct EqNode
	{
		bool operator() (const StateTimeAStarNode* n1, const StateTimeAStarNode* n2) const
		{
			return (n1 == n2) ||
				(n1 && n2 && n1->state.location == n2->state.location && 
				n1->state.orientation == n2->state.orientation && n1->goal_id == n2->goal_id);
		}
	};

	fibonacci_heap< StateTimeAStarNode*, compare<StateTimeAStarNode::compare_node> > open_list;
	unordered_set< StateTimeAStarNode*, Hasher, EqNode> allNodes_table;
	// generate start and add it to the OPEN list
	double h_val = path_planner.compute_h_value(G, start.location, 0, goal_locations);
	auto root = new StateTimeAStarNode(start, 0, h_val, nullptr, 0);

	num_generated++;
	root->open_handle = open_list.push(root);
	root->in_openlist = true;
	allNodes_table.insert(root);

	while (!open_list.empty())
	{
		auto* curr = open_list.top();
		open_list.pop();
		curr->in_openlist = false;
		num_expanded++;

		// check if the popped node is a goal
		if (curr->state.location == goal_locations[curr->goal_id].first &&
			curr->state.timestep >= goal_locations[curr->goal_id].second) // reach the goal location after its release time
		{
			curr->goal_id++;
			if (curr->goal_id == (int)goal_locations.size())
			{
				Path path(curr->state.timestep + 1);
				for (int t = curr->state.timestep; t >= 0; t--)
				{
					path[t] = curr->state;
					curr = curr->parent;
				}
				open_list.clear();
				for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
					delete (*it);
				allNodes_table.clear();
				return path;
			}
		}

		double travel_time = 1;
		auto p = travel_times.find(curr->state.location);
		if (p != travel_times.end())
		{
			travel_time += p->second;
		}
		for (const auto& next_state : G.get_neighbors(curr->state))
		{
			if (curr->state.location == next_state.location && curr->state.orientation == next_state.orientation)
				continue;
			// compute cost to next_id via curr node
			double next_g_val = curr->g_val + G.get_weight(curr->state.location, next_state.location) * travel_time;
			double next_h_val = path_planner.compute_h_value(G, next_state.location, curr->goal_id, goal_locations);
			if (next_h_val >= INT_MAX) // This vertex cannot reach the goal vertex
				continue;

			// generate (maybe temporary) node
			auto next = new StateTimeAStarNode(next_state, next_g_val, next_h_val, curr, 0);

			// try to retrieve it from the hash table
			auto existing = allNodes_table.find(next);
			if (existing == allNodes_table.end())
			{
				next->open_handle = open_list.push(next);
				next->in_openlist = true;
				num_generated++;
				allNodes_table.insert(next);
			}
			else
			{  // update existing node's if needed (only in the open_list)

				if ((*existing)->getFVal() > next->getFVal())
				{
					// update existing node
					(*existing)->g_val = next_g_val;
					(*existing)->h_val = next_h_val;
					(*existing)->goal_id = next->goal_id;
					(*existing)->parent = curr;
					(*existing)->depth = next->depth;
					if ((*existing)->in_openlist)
					{
						open_list.increase((*existing)->open_handle);  // increase because f-val improved*/
					}
					else // re-open
					{
						(*existing)->open_handle = open_list.push(*existing);
						(*existing)->in_openlist = true;
					}
				}
				delete(next);  // not needed anymore -- we already generated it before

			}  // end update an existing node
		}  // end for loop that generates successors
	}  // end while loop

	open_list.clear();
	for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete (*it);
	allNodes_table.clear();
	return Path();
}



void LRAStar::resolve_conflicts(const vector<Path>& input_paths)
{
    num_wait_commands = 0;
    num_of_agents = input_paths.size();
    vector<int> path_pointers(num_of_agents, 1);
	solution.clear();
	solution.resize(num_of_agents);
    curr_locations.clear();
    for (int k = 0; k < num_of_agents; k++)
    {
		// solution[k].reserve(window + 1);
		solution[k].push_back(input_paths[k][0]);
        if (k_robust == 1)
            curr_locations[input_paths[k][0].location] = k;

    }

    for (int t = 1; t <= simulation_window; t++)
    {
        next_locations.clear();
        vector<int> agents_list(num_of_agents);
        for (int k = 0; k < num_of_agents; k++)
        {
            agents_list[k] = k;
        }
        std::random_shuffle(agents_list.begin(), agents_list.end());
        for (auto agent : agents_list)
        {
			if (path_pointers[agent] >= (int) input_paths[agent].size())
			{
				path_pointers[agent] = (int) input_paths[agent].size() - 1;
			}
            int loc = input_paths[agent][path_pointers[agent]].location;
            int orientation = input_paths[agent][path_pointers[agent]].orientation;
            if (loc == solution[agent][t - 1].location)
            { // The agent wait or rotates at its current location
				solution[agent].emplace_back(loc, t, orientation);
                path_pointers[agent]++;
                auto other = next_locations.find(loc); // conflict with other agent
                if (other != next_locations.end())
                {
                    wait_command(other->second, t, path_pointers); // Other agent has to wait
                    path_pointers[other->second]--;
                }
            }
            else if (curr_locations.find(loc) != curr_locations.end())
            { // The agent cannot move because its next location is occupied currently
                wait_command(agent, t, path_pointers);
            }
            else // The agent wants to move to a location that is currently empty
            {
                auto other = next_locations.find(loc);
                if (other == next_locations.end())
                { // No other agents want to go to this location yet
					solution[agent].emplace_back(loc, t, orientation);
                    path_pointers[agent]++;
                }
                else
                { // Another agent also wants to go to this location, for now we just force this agent to wait
                    wait_command(agent, t, path_pointers);
                }
            }
            next_locations[solution[agent][t].location] = agent;
        }
        if (k_robust == 1)
            curr_locations = next_locations;
    }
    print_results();
}


void LRAStar::wait_command(int agent, int timestep,
        vector<list<pair<int, int> >::const_iterator >& traj_pointers)
{
    int location = solution[agent][timestep - 1].location;
    if ((int)solution[agent].size() == timestep)
    {
		solution[agent].push_back(solution[agent][timestep - 1]);
    }
    else
    {
		solution[agent][timestep] = solution[agent][timestep - 1];
    }
	solution[agent][timestep].timestep = timestep;
    auto other = next_locations.find(location); // whether conflict with other agent
    if (other != next_locations.end())
    {
        wait_command(other->second, timestep, traj_pointers); // Other agent has to wait
        --traj_pointers[other->second];
    }
    next_locations[location] = agent;
    num_wait_commands++;
}


void LRAStar::wait_command(int agent, int timestep,
                           vector<int>& path_pointers)
{
    int location = solution[agent][timestep - 1].location;
    if ((int)solution[agent].size() == timestep)
    {
		solution[agent].push_back(solution[agent][timestep - 1]);
    }
    else
    {
		solution[agent][timestep] = solution[agent][timestep - 1];
    }
	solution[agent][timestep].timestep = timestep;
    auto other = next_locations.find(location); // whether conflict with other agent
    if (other != next_locations.end())
    {
        wait_command(other->second, timestep, path_pointers); // Other agent has to wait
        path_pointers[other->second]--;
    }
    next_locations[location] = agent;
    num_wait_commands++;
}


void LRAStar::print_results() const
{
	if (num_wait_commands > 0)
		 std::cout << "LRA*:Succeed," << runtime << "," <<
              num_wait_commands << "," <<
              num_expanded << "," << num_generated <<
              std::endl;
}

void LRAStar::save_results(const string &fileName, const string &instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << "," <<
          num_wait_commands << "," << num_wait_commands << "," <<
          num_expanded << "," << num_generated << "," <<
          0 << "," << 0 << "," <<
          0 << "," << 0 << "," <<
          instanceName << std::endl;
    stats.close();
}