#include "MAPFSolver.h"
#include <ctime>
#include <iostream>
#include "PathTable.h"


MAPFSolver::MAPFSolver(const BasicGraph& G, SingleAgentSolver& path_planner):
        solution_found(false), solution_cost(-2),
        avg_path_length(-1), G(G), path_planner(path_planner), initial_rt(G), rt(G) {}


MAPFSolver::~MAPFSolver()
{
}

// TODO: implement validate_solution function
bool MAPFSolver::validate_solution()
{
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
           /* find_conflicts(conflict, a1, a2);
            if (!conflict.empty())
            {
                int a1, a2, loc1, loc2, t;
                std::tie(a1, a2, loc1, loc2, t) = conflict.front();
                if (loc2 < 0)
                    std::cout << "Agents "  << a1 << " and " << a2 << " collides at " << loc1 <<
                    " at timestep " << t << std::endl;
                else
                    std::cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
                              loc1 << "-->" << loc2 << ") at timestep " << t << std::endl;
                return false;
            }*/
		}
	}
	return true;
}


void MAPFSolver::print_solution() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        cout << "Agent " << i << ":\t";
        for (const auto & loc : solution[i])
        {
            cout << loc.location << ",";
        }
        cout << endl;
    }
}