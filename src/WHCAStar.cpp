#include "WHCAStar.h"

WHCAStar::WHCAStar(const BasicGraph &G, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner) {}


bool WHCAStar::run(const vector<State>& starts,
                   const vector< vector<pair<int, int> > >& goal_locations,
                   int time_limit)
{
    // set timer
    clock_t start = std::clock();
    num_expanded = 0;
    num_generated = 0;
    num_restarts = 0;
    int num_of_agents = starts.size();

    ReservationTable rt(G);
    rt.num_of_agents = num_of_agents;
    rt.map_size = G.size();
    rt.k_robust = k_robust;
    rt.window = window;
	rt.hold_endpoints = hold_endpoints;
    // path_planner.window = window;
    rt.use_cat = false;
    rt.prioritize_start = false;
    path_planner.prioritize_start = false;
	path_planner.hold_endpoints = hold_endpoints;
    path_planner.travel_times.clear();

    std::vector<int> priorities(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
        priorities[i] = i;

    runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
    while (runtime < time_limit)
    {
        num_restarts++;
        // generate random priority order
        std::random_shuffle(priorities.begin(), priorities.end());

        solution_cost = 0;
        solution = initial_solution;
        bool succ = true;
        for (int i : priorities)
        {
			rt.copy(initial_rt);
            rt.build(solution, initial_constraints, i);
			solution[i] = path_planner.run(G, starts[i], goal_locations[i], rt);
            solution_cost += path_planner.path_cost;
            rt.clear();
            num_expanded += path_planner.num_expanded;
            num_generated += path_planner.num_generated;
            runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
            if (solution[i].empty() || runtime >= time_limit)
            {
                succ = false;
                break;
            }
        }
        if (succ)
        {
            runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
            min_sum_of_costs = 0;
            for (int i = 0; i < num_of_agents; i++)
            {
                int start = starts[i].location;
                for (const auto& goal : goal_locations[i])
                {
                    min_sum_of_costs += G.heuristics.at(goal.first)[start];
                    start = goal.first;
                }
            }
            avg_path_length = 0;
            for (int k = 0; k < num_of_agents; k++)
            {
                avg_path_length += (int)solution[k].size();
            }
            avg_path_length /= num_of_agents;
            solution_found = true;
            print_results();
            return true;
        }
    }
    runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
    solution_cost = -1;
    solution_found = false;
    print_results();
    return false;
}

void WHCAStar::print_results() const
{
    std::cout << "WHCA*:";
    if(solution_cost >= 0) // solved
        std::cout << "Succeed,";
    else // time_out
        std::cout << "Timeout,";

    std::cout << runtime << "," <<
                num_restarts << "," <<
                num_expanded << "," << num_generated << "," <<
                solution_cost << "," << min_sum_of_costs << "," <<
                avg_path_length <<
    std::endl;
}


void WHCAStar::save_results(const std::string &fileName, const std::string &instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << "," <<
            num_restarts << "," << num_restarts << "," <<
            num_expanded << "," << num_generated << "," <<
            solution_cost << "," << min_sum_of_costs << "," <<
            avg_path_length << "," << "0" << "," <<
            instanceName << std::endl;
    stats.close();
}


void WHCAStar::clear()
{
	runtime = 0;
	solution_found = false;
	solution_cost = -2;
	avg_path_length = -1;
	num_expanded = 0;
	num_generated = 0;
	num_restarts = 0;
	solution.clear();
	initial_constraints.clear();
	initial_rt.clear();
}