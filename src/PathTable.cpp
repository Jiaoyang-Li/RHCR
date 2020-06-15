#include "PathTable.h"


PathTable::PathTable(const vector<Path*>& paths, int window, int k_robust):
    window(window), k_robust(k_robust)
{
    num_of_agents = (int)paths.size();
    for (int i = 0; i < num_of_agents; i++)
    {
        for (auto state : (*paths[i]))
        {
            if (state.timestep > window)
                break;
            PT[state.location].emplace_back(state.timestep, i);
        }
    }
}


void PathTable::remove(const Path* old_path, int agent)
{
    if (old_path == nullptr)
        return;
    for (auto state : (*old_path))
    {
        if (state.timestep > window)
            break;
        for (auto it = PT[state.location].begin(); it != PT[state.location].end(); ++it)
        {
            if (it->first == state.timestep && it->second == agent)
            {
                PT[state.location].erase(it);
                break;
            }
        }
    }
}


list<std::shared_ptr<Conflict> > PathTable::add(const Path* new_path, int agent)
{
    list<std::shared_ptr<Conflict> > conflicts;
    vector<bool> conflicting_agents(num_of_agents, false);
    for (auto state : (*new_path))
    {
        if (state.timestep > window)
            break;
        for (auto it = PT[state.location].begin(); it != PT[state.location].end(); ++it)
        {
            if (conflicting_agents[it->second])
                continue;
            else if (abs(it->first - state.timestep) <= k_robust)
            {
                conflicts.push_back(std::shared_ptr<Conflict>(
                        new Conflict(agent, it->second, state.location, -1, min(it->first, state.timestep))));
                conflicting_agents[it->second] = true;
            }
        }
    }
    for (auto state : (*new_path))
    {
        if (state.timestep > window)
            break;
        PT[state.location].emplace_back(state.timestep, agent);
    }
    return conflicts;
}