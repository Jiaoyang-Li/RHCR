#include "ECBS.h"

void ECBS::clear()
{
    runtime = 0;
    HL_num_expanded = 0;
    HL_num_generated = 0;
    LL_num_expanded = 0;
    LL_num_generated = 0;
    solution_found = false;
    solution_cost = -2;
    min_f_val = -1;
    focal_threshold = -1;
    avg_path_length = -1;
    paths.clear();
    path_min_costs.clear();
    path_costs.clear();
    open_list.clear();
    focal_list.clear();
    release_closed_list();
    starts.clear();
    goal_locations.clear();
}

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
void ECBS::update_paths(ECBSNode* curr)
{
    vector<bool> updated(num_of_agents, false);  // initialized for false
    while (curr != nullptr)
    {
        for (auto p = curr->paths.begin(); p != curr->paths.end(); ++p)
        {
            int agent = std::get<0>(*p);
            if (!updated[agent])
            {
                paths[agent] = &(std::get<1>(*p));
                path_min_costs[agent] = std::get<2>(*p);
                path_costs[agent] = std::get<3>(*p);
                updated[agent] = true;
            }
        }
        curr = curr->parent;
    }
}


/*void ECBS::collect_constraints(ECBSNode* curr, int current_agent)
{
    // extract all constraints on agent_id
    std::list <Constraint > constraints;
    int max_timestep = k_robust + 1;
    while (curr != dummy_start)
    {
        for (Constraint c : curr->constraints)
        {
            if (std::get<0>(c) == current_agent)
            {
                constraints.push_back(c);
                max_timestep = std::max(max_timestep, std::get<3>(c));
            }
        }
        curr = curr->parent;
    }

    // initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
    constraint_table.clear();
    constraint_table.resize(max_timestep + 1);

    // add initial constraints
    for (auto con : initial_constraints)
    {
        if (std::get<0>(con) != current_agent)
        {
            for (int t = 0; t < std::get<2>(con); t++)
                constraint_table[t].insert(std::make_pair(std::get<1>(con), -1));
        }
    }


    for (Constraint c : constraints)
    {
        (constraint_table)[std::get<3>(c)].insert(std::make_pair(std::get<1>(c), std::get<2>(c)));
    }
}*/



// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void ECBS::copy_conflicts(const std::list<std::shared_ptr<Conflict>>& conflicts,
                            std::list<std::shared_ptr<Conflict> >& copy,
                            const list<int>& excluded_agents) const
{
    for (auto it = conflicts.begin(); it != conflicts.end(); ++it)
    {
        bool to_copy = true;
        for (int a : excluded_agents)
        {
            if (a == std::get<0>(**it) || a == std::get<1>(**it))
            {
                to_copy = false;
                break;
            }
        }
        if (to_copy)
        {
            copy.push_back(*it);
        }
    }
}


void ECBS::find_conflicts(int start_time, std::list<std::shared_ptr<Conflict> >& conflicts, int a1, int a2) const
{
    if (paths[a1] == nullptr || paths[a2] == nullptr)
        return;
	if (hold_endpoints)
	{
		// TODO: add k-robust
		size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
		for (size_t timestep = start_time; timestep < min_path_length; timestep++)
		{
			int loc1 = paths[a1]->at(timestep).location;
			int loc2 = paths[a2]->at(timestep).location;
			if (loc1 == loc2)
			{
				conflicts.emplace_back(new Conflict(a1, a2, loc1, -1, timestep));
				return;
			}
			else if (timestep < min_path_length - 1
				&& loc1 == paths[a2]->at(timestep + 1).location
				&& loc2 == paths[a1]->at(timestep + 1).location)
			{
				conflicts.emplace_back(new Conflict(a1, a2, loc1, loc2, timestep + 1)); // edge conflict
				return;
			}
		}

		if (paths[a1]->size() != paths[a2]->size())
		{
			int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
			int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
			int loc1 = paths[a1_]->back().location;
			for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
			{
				int loc2 = paths[a2_]->at(timestep).location;
				if (loc1 == loc2)
				{
					conflicts.emplace_back(new Conflict(a1_, a2_, loc1, -1, timestep)); // It's at least a semi conflict		
					return;
				}
			}
		}
	}
	else
	{
		int size1 = min(window + 1, (int)paths[a1]->size());
		int size2 = min(window + 1, (int)paths[a2]->size());
		for (int timestep = start_time; timestep < size1; timestep++)
		{
			if (size2 <= timestep - k_robust)
				break;
			else if (k_robust > 0)
			{
				int loc = paths[a1]->at(timestep).location;
				for (int i = max(0, timestep - k_robust); i <= min(timestep + k_robust, size2 - 1); i++)
				{
					if (loc == paths[a2]->at(i).location)
					{
						conflicts.emplace_back(new Conflict(a1, a2, loc, -1, min(i, timestep))); // k-robust vertex conflict
						return;
					}
				}

			}
			else
			{

				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					conflicts.emplace_back(new Conflict(a1, a2, loc1, -1, timestep));
					return;
				}
				else if (timestep < size1 - 1 && timestep < size2 - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					conflicts.emplace_back(new Conflict(a1, a2, loc1, loc2, timestep + 1)); // edge conflict
					return;
				}
			}

		}
	}

}

void ECBS::find_conflicts(int start_time, std::list<std::shared_ptr<Conflict> >& conflicts) const
{
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            find_conflicts(start_time, conflicts, a1, a2);
        }
    }
}

void ECBS::find_conflicts(std::list<std::shared_ptr<Conflict> >& new_conflicts, int new_agent) const
{
    for (int a2 = 0; a2 < num_of_agents; a2++)
    {
        if(new_agent == a2)
            continue;
        find_conflicts(0, new_conflicts, new_agent, a2);
    }
}



void ECBS::find_conflicts(const std::list<std::shared_ptr<Conflict> >& old_conflicts,
                            std::list<std::shared_ptr<Conflict> >& new_conflicts,
                            const list<int>& new_agents) const
{
    // Copy from parent
    copy_conflicts(old_conflicts, new_conflicts, new_agents);

    // detect new conflicts
    for (int a : new_agents)
        find_conflicts(new_conflicts, a);
}

void ECBS::remove_conflicts(std::list<std::shared_ptr<Conflict> >& conflicts, int excluded_agent) const
{
    for (auto it = conflicts.begin(); it != conflicts.end(); ++it)
    {
        if(std::get<0>(**it) == excluded_agent || std::get<1>(**it) == excluded_agent)
        {
            it = conflicts.erase(it);
            --it;
        }
    }
}

void ECBS::choose_conflict(ECBSNode &node) const
{
    if (node.conflicts.empty())
        return;

    node.conflict = node.conflicts.front();

    // choose the earliest
    for (auto conflict : node.conflicts)
    {
        if (std::get<4>(*conflict) < std::get<4>(*node.conflict))
            node.conflict = conflict;
    }
}


/*double ECBS::get_path_cost(const Path& path) const
{
    double cost = 0;
    for (int i = 0; i < (int)path.size() - 1; i++)
    {
        double travel_time = 1;
        if (i > window && !travel_times.empty())
        {
            int dir = G.get_direction(path[i].location, path[i + 1].location);
            travel_time += travel_times[path[i].location][dir];
        }
        cost += G.get_weight(path[i].location, path[i + 1].location) * travel_time;
    }
    return cost;
}*/


bool ECBS::find_path(ECBSNode* node, int agent)
{
    Path path;

    // extract all constraints on the agent
    list<Constraint> constraints;
    ECBSNode* curr = node;
    while (curr != dummy_start)
    {
        for (Constraint c : curr->constraints)
        {
            if (disjoint_splitting)
            {
                if (std::get<0>(c) == agent)
                    constraints.push_back(c);
                else if (std::get<4>(c)) // positive constraint on other agents
                {
                    int loc = std::get<1>(c);
                    int time = std::get<3>(c);
                    int t_min = max(0, time - k_robust);
                    int t_max = min(window, time + k_robust);
                    for (int t = t_min; t <= t_max; t++)
                    {
                        constraints.emplace_back(agent, loc, -1, t, false);
                    }
                }
            }
            else if (std::get<0>(c) == agent)
            {
                constraints.push_back(c);
            }
        }
        curr = curr->parent;
    }
	rt.copy(initial_rt);
    rt.build(paths, initial_constraints, constraints, agent);

    path = path_planner.run(G, starts[agent], goal_locations[agent], rt);
    rt.clear();
    LL_num_expanded += path_planner.num_expanded;
    LL_num_generated += path_planner.num_generated;

    if (path.empty())
    {
        if (screen == 2)
            std::cout << "Fail to find a path" << std::endl;
        return false;
    }
    else if (screen == 2 && !validate_path(path, constraints))
    {
        std::cout << "The resulting path violates its constraints!" << endl;
        cout << path << endl;
        for (auto constraint : constraints)
            cout << constraint << endl;
		rt.copy(initial_rt);
        rt.build(paths, list< tuple<int, int, int> >(), constraints, agent);
        path = path_planner.run(G, starts[agent], goal_locations[agent], rt);
        rt.clear();
        exit(-1);
    }
    node->g_val = node->g_val - path_costs[agent] + path_planner.path_cost;
    node->min_f_val = node->min_f_val - path_min_costs[agent] + path_planner.min_f_val;
    for (auto it = node->paths.begin(); it != node->paths.end(); ++it)
    {
        if (std::get<0>(*it) == agent)
        {
            node->paths.erase(it);
            break;
        }
    }
    node->paths.emplace_back(agent, path, path_planner.min_f_val, path_planner.path_cost);
    paths[agent] = &std::get<1>(node->paths.back());
    return true;
}


bool ECBS::validate_path(const Path& path, const list<Constraint>& constraints) const
{
    int a, v1, v2, t;
    bool positive;
    for (auto constraint : constraints)
    {
        std::tie(a, v1, v2, t, positive) = constraint;
        if (positive)
        {
            if (path[t].location != v1)
                return false;
        }
        else if (v2 < 0)
        {
            if (path[t].location == v1)
                return false;
        }
        else
        {
            if (path[t - 1].location == v1 && path[t].location == v2)
                return false;
        }
    }
    return true;
}


bool ECBS::generate_child(ECBSNode* node, ECBSNode* parent)
{
    list<int> to_replan;
    int agent, v1, v2, time;
    bool positive;
    std::tie(agent, v1, v2, time, positive) = node->constraints.front();
    if (positive) // positive vertex constraint
    {
        int t_min = max(0, time - k_robust);
        for (int i = 0; i < num_of_agents; i++)
        {
            if (i == agent)
                continue;
            int t_max = min(min(window, time + k_robust), (int)paths[i]->size() - 1);
            for (int t = t_min; t <= t_max; t++)
            {
                if (paths[i]->at(t).location == v1)
                {
                    to_replan.emplace_back(i);
                    break;
                }
            }
        }
    }
    else
    {
        to_replan.emplace_back(agent);
    }

    for (int a : to_replan)
    {
        if (!find_path(node, a))
            return false;
    }
    find_conflicts(node->parent->conflicts, node->conflicts, to_replan);
	node->window = window;
    node->num_of_collisions = node->conflicts.size();

    // Estimate h value
    node->h_val = 0;
    node->f_val = node->g_val + node->h_val;

    return true;
}


bool ECBS::generate_root_node()
{
    clock_t time = std::clock();
    dummy_start = new ECBSNode();

    // initialize paths_found_initially
    paths.resize(num_of_agents, nullptr);
    path_min_costs.resize(num_of_agents, 0);
    path_costs.resize(num_of_agents, 0);

    if (screen == 2)
        std::cout << "Generate root CT node ..." << std::endl;


    for (int i = 0; i < num_of_agents; i++)
    {
		rt.copy(initial_rt);
        rt.build(paths, initial_constraints, list<Constraint>(), i);
        Path path = path_planner.run(G, starts[i], goal_locations[i], rt);
        /*if (path.empty() && hold_endpoints && goal_locations[i].size() == 1)
        {
			goal_locations[i][0] = starts[i].location;
			Path path = path_planner.run(G, starts[i], goal_locations[i], rt);
		}*/

		if (path.empty())
		{
			std::cout << "NO SOLUTION EXISTS";
			return false;
        }

		rt.clear();
		LL_num_expanded += path_planner.num_expanded;
		LL_num_generated += path_planner.num_generated;


        dummy_start->paths.emplace_back(i, path, path_planner.min_f_val, path_planner.path_cost);
        paths[i] = &std::get<1>(dummy_start->paths.back());
        path_min_costs[i] = path_planner.min_f_val;
        path_costs[i] = path_planner.path_cost;
        dummy_start->g_val += path_planner.path_cost;
        dummy_start->min_f_val += path_planner.min_f_val;
    }
    find_conflicts(0, dummy_start->conflicts);
	dummy_start->window = window;
    dummy_start->f_val = dummy_start->g_val;
    dummy_start->num_of_collisions = dummy_start->conflicts.size();
    min_f_val = dummy_start->min_f_val;
    focal_threshold = min_f_val * suboptimal_bound;
    push_node(dummy_start);
    best_node = dummy_start;
    if (screen == 2)
    {
        runtime = (std::clock() - time) * 1.0 / CLOCKS_PER_SEC;
        std::cout << "Done! (" << runtime << "s)" << std::endl;
    }
    return true;
}


void ECBS::push_node(ECBSNode* node)
{
    node->open_handle = open_list.push(node);
    if (node->f_val <= focal_threshold)
        node->focal_handle = focal_list.push(node);
    allNodes_table.push_back(node);
}

void ECBS::reinsert_node(ECBSNode* node)
{
	node->open_handle = open_list.push(node);
	if (node->f_val <= focal_threshold)
		node->focal_handle = focal_list.push(node);
}

ECBSNode* ECBS::pop_node()
{
    update_focal_list();
    ECBSNode* node = focal_list.top();
    focal_list.pop();
    open_list.erase(node->open_handle);
    node->in_openlist = false;
    return node;
}


bool ECBS::run(const std::vector<State>& starts,
                     const std::vector< vector<pair<int, int> > >& goal_locations,
                     int time_limit)
{
    clear();

    // set timer
    start = std::clock();

    this->starts = starts;
    this->goal_locations = goal_locations;
    this->num_of_agents = starts.size();
    this->time_limit = time_limit;

    solution_cost = -2;
    solution_found = false;

    rt.num_of_agents = num_of_agents;
    rt.map_size = G.size();
    rt.k_robust = k_robust;
    rt.window = window;
	rt.hold_endpoints = hold_endpoints;
    // path_planner.window = window;
    rt.use_cat = true;
	path_planner.suboptimal_bound = suboptimal_bound;
    path_planner.prioritize_start = false;
	path_planner.hold_endpoints = hold_endpoints;
    rt.prioritize_start = false;
    path_planner.travel_times.clear();

    if (!generate_root_node())
        return false;

    // start the loop
    while (!open_list.empty() && !solution_found)
    {
        runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
        if (runtime > time_limit)
        {  // timeout
            solution_cost = -1;
            solution_found = false;
            break;
        }

        ECBSNode* curr = pop_node();
        update_paths(curr);

		if (window > curr->window)
		{
			find_conflicts(curr->window, curr->conflicts);
			curr->window = window;
			curr->num_of_collisions = (int)curr->conflicts.size();
			reinsert_node(curr);
			continue;
		}

        if (curr->conflicts.empty())
        {// found a solution (and finish the while look)
			if (potential_function == "SOC")
			{
				double soc = 0;
				for (int i = 0; i < num_of_agents; i++)
				{
					soc += path_planner.compute_h_value(G, paths[i]->front().location, 0, goal_locations[i]);
					soc -= max((int)paths[i]->size() - window, 0);
				}
				if (soc <= 0)
				{
					window++;
					rt.window++;
					find_conflicts(curr->window, curr->conflicts);
					curr->window = window;
					curr->num_of_collisions = (int)curr->conflicts.size();
					reinsert_node(curr);
					continue;
				}
			}
			else if (potential_function == "IC")
			{
				int count = 0;
				for (int i = 0; i < num_of_agents; i++)
				{
					if (path_planner.compute_h_value(G, paths[i]->front().location, 0, goal_locations[i]) <= max((int)paths[i]->size() - window, 0))
						count++;
				}
				if (count > num_of_agents * potential_threshold)
				{
					window++;
					rt.window++;
					find_conflicts(curr->window, curr->conflicts);
					curr->window = window;
					curr->num_of_collisions = (int)curr->conflicts.size();
					reinsert_node(curr);
					continue;
				}
			}
			solution_found = true;
			best_node = curr;
			solution_found = true;
			solution_cost = curr->g_val;
			break;
        }

        choose_conflict(*curr);
        if (std::get<4>(*curr->conflict) > std::get<4>(*best_node->conflict))
            best_node = curr;
        else if (std::get<4>(*curr->conflict) == std::get<4>(*best_node->conflict) &&
                 curr->f_val < best_node->f_val)
            best_node = curr;

        //Expand the node
        HL_num_expanded++;

        curr->time_expanded = HL_num_expanded;
        if(screen == 2)
            std::cout << "Expand Node " << curr->time_generated << " ( cost = " << curr->f_val << " , min_cost = " <<
                      curr->min_f_val << ", #conflicts = " <<
                      curr->num_of_collisions << " ) on conflict " << *curr->conflict << std::endl;
        ECBSNode* n[2];
        for (int i = 0; i < 2; i++)
            n[i] = new ECBSNode(curr);
        resolve_conflict(*curr->conflict, n[0], n[1]);

        vector<Path*> copy(paths);
        for (int i = 0; i < 2; i++)
        {
            bool sol = generate_child(n[i], curr);
            if (sol)
            {
                HL_num_generated++;
                n[i]->time_generated = HL_num_generated;
            }
            if (sol)
            {
                if (screen == 2)
                {
                    std::cout << "Generate #" << n[i]->time_generated << " with "
                              << n[i]->paths.size() << " new paths, "
                              << n[i]->g_val - curr->g_val << " delta cost and "
                              << n[i]->num_of_collisions << " conflicts " << std::endl;
                }
                push_node(n[i]);
            }
            else
            {
                delete (n[i]);
                n[i] = nullptr;
            }
            paths = copy;
        }
    }  // end of while loop


    runtime = (std::clock() - start) * 1.0 / CLOCKS_PER_SEC;
    get_solution();
    if (solution_found && !validate_solution())
    {
        std::cout << "Solution invalid!!!" << std::endl;
        // print_paths();
        exit(-1);
    }

    if (screen > 0) // 1 or 2
        print_results();
    return solution_found;
}


void ECBS::resolve_conflict(const Conflict& conflict, ECBSNode* n1, ECBSNode* n2)
{
    int a1, a2, v1, v2, t;
    std::tie(a1, a2, v1, v2, t) = conflict;
    if (disjoint_splitting)
    {
        if (v2 < 0) // vertex conflict
        {
            n1->constraints.emplace_back(a1, v1, v2, t, true);
            n2->constraints.emplace_back(a1, v1, v2, t, false);
            /*for (int i = max(0, t - k_robust); i <= min(t + k_robust, (int)paths[a1]->size() - 1); i++)
            {
                if (paths[a1]->at(i).location == v1)
                {
                    n1->constraints.emplace_back(a1, v1, v2, i, true);
                    n2->constraints.emplace_back(a2, v1, v2, i, false);
                    break;
                }
            }*/
        }
        else // TODO:: edge conflict
        {
        }
    }
    else
    {
        if (v2 < 0) // vertex conflict
        {
            for (int i = 0; i <= k_robust; i++)
            {
                n1->constraints.emplace_back(a1, v1, v2, t + i, false);
                n2->constraints.emplace_back(a2, v1, v2, t + i, false);
            }
        }
        else // edge conflict
        {
            n1->constraints.emplace_back(a1, v1, v2, t, false);
            n2->constraints.emplace_back(a2, v2, v1, t, false);
        }
    }

}



ECBS::ECBS(const BasicGraph& G, SingleAgentSolver& path_planner):
        MAPFSolver(G, path_planner), disjoint_splitting(false) {}


void ECBS::release_closed_list()
{
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
        delete *it;
    allNodes_table.clear();
}


ECBS::~ECBS()
{
    release_closed_list();
}


bool ECBS::validate_solution() const
{
    list<std::shared_ptr<Conflict>> conflict;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            find_conflicts(0, conflict, a1, a2);
            if (!conflict.empty())
            {
                int a1, a2, loc1, loc2, t;
                std::tie(a1, a2, loc1, loc2, t) = *(conflict.front());
                if (loc2 < 0)
                    std::cout << "Agents "  << a1 << " and " << a2 << " collide at " << loc1 <<
                              " at timestep " << t << std::endl;
                else
                    std::cout << "Agents " << a1 << " and " << a2 << " collide at (" <<
                              loc1 << "-->" << loc2 << ") at timestep " << t << std::endl;
                return false;
            }
        }
    }
    return true;
}

void ECBS::print_paths() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        if (paths[i] == nullptr)
            continue;
        std::cout << "Agent " << i << " (" << paths[i]->size() - 1 << "): ";
        for (auto s : (*paths[i]))
            std::cout << s.location << "->";
        std::cout << std::endl;
    }
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ECBS::update_focal_list()
{
    ECBSNode* open_head = open_list.top();
    if (open_head->min_f_val > min_f_val)
    {
        if (screen == 2)
        {
            std::cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
        }
        min_f_val = open_head->min_f_val;
        double new_focal_list_threshold = min_f_val * suboptimal_bound;
        for (ECBSNode* n : open_list)
        {
            if (n->f_val > focal_threshold &&
                n->f_val <= new_focal_list_threshold)
                n->focal_handle = focal_list.push(n);
        }
        focal_threshold = new_focal_list_threshold;
        if (screen == 2)
        {
            std::cout << focal_list.size() << std::endl;
        }
    }
}

void ECBS::print_results() const
{
    std::cout << "ECBS:";
    if(solution_cost >= 0) // solved
        std::cout << "Succeed,";
    else if(solution_cost == -1) // time_out
        std::cout << "Timeout,";
    else if(solution_cost == -2) // no solution
        std::cout << "No solutions,";
    else if (solution_cost == -3) // nodes out
        std::cout << "Nodesout,";

    std::cout << runtime << "," <<
              HL_num_expanded << "," << HL_num_generated << "," <<
              LL_num_expanded << "," << LL_num_generated << "," <<
              solution_cost << "," << min_f_val << "," <<
              avg_path_length << "," << dummy_start->num_of_collisions << "," <<
			  window <<
              std::endl;
}

void ECBS::save_results(const std::string &fileName, const std::string &instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << "," <<
          HL_num_expanded << "," << HL_num_generated << "," <<
          LL_num_expanded << "," << LL_num_generated << "," <<
          solution_cost << "," << min_f_val << "," <<
          avg_path_length << "," << dummy_start->num_of_collisions << "," <<
		  instanceName << "," << window << std::endl;
    stats.close();
}

void ECBS::print_conflicts(const ECBSNode &curr) const
{
    for (auto c : curr.conflicts)
    {
        std::cout << *c << std::endl;
    }
}


void ECBS::save_search_tree(const std::string &fname) const
{
    std::ofstream output;
    output.open(fname, std::ios::out);
    output << "digraph G {" << std::endl;
    output << "size = \"5,5\";" << std::endl;
    output << "center = true;" << std::endl;
    for (auto node : allNodes_table)
    {
        if (node == dummy_start)
            continue;
        else if (node->time_expanded == 0) // this node is in the openlist
            output << node->time_generated << " [color=blue]" << std::endl;
        output << node->parent->time_generated << " -> " << node->time_generated << std::endl;
    }
    output << "}" << std::endl;
    output.close();
}


void ECBS::get_solution()
{
    update_paths(best_node);
    solution.resize(num_of_agents);
    for (int k = 0; k < num_of_agents; k++)
    {
        solution[k] = *paths[k];
    }

    //solution_cost  = 0;
    avg_path_length = 0;

    for (int k = 0; k < num_of_agents; k++)
    {
        /*if (k == 250)
            cout << solution[k] << endl;
        solution[k].clear();
        rt.build(solution, initial_constraints, k);
        vector< vector<double> > h_values(goal_locations[k].size());
        for (int j = 0; j < (int) goal_locations[k].size(); j++)
        {
            h_values[j] = G.heuristics.at(goal_locations[k][j]);
        }
        solution[k] = path_planner.run(G, starts[k], goal_locations[k], rt, h_values);
        if (solution[k].empty())
            cout << "ERROR" << endl;
        rt.clear();
        LL_num_expanded += path_planner.num_expanded;
        LL_num_generated += path_planner.num_generated;*/
        //solution_cost += path_planner.path_cost;
        avg_path_length += paths[k]->size();
    }
    avg_path_length /= num_of_agents;
}