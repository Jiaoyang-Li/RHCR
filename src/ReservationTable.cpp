#include "ReservationTable.h"

// update SIT at the given location
void ReservationTable::updateSIT(size_t location)
{
	if (sit.find(location) == sit.end())
	{
		const auto& it = ct.find(location);
		if (it != ct.end())
		{
			for (auto time_range : it->second)
				insertConstraint2SIT(location, time_range.first, time_range.second);
			ct.erase(it);
		}

		if (location < map_size) // vertex
		{
			for (int t = 0; t < (int)cat.size(); t++)
			{
				if (cat[t][location])
				{
					insertSoftConstraint2SIT(location, t, t + 1);
				}
			}
		}
		else // edge
		{
			auto edge = getEdge(location);
			for (int t = 1; t < (int)cat.size(); t++)
			{
				if (cat[t][edge.first] && cat[t - 1][edge.second])
				{
					insertSoftConstraint2SIT(location, t, t + 1);
				}
			}
		}
	}
}


//merge successive safe intervals with the same number of conflicts.
void ReservationTable::mergeIntervals(list<Interval >& intervals) const
{
	if (intervals.empty())
		return;
	auto prev = intervals.begin();
	auto curr = prev;
	++curr;
	while (curr != intervals.end())
	{
		if (std::get<1>(*prev) == std::get<0>(*curr) && std::get<2>(*prev) == std::get<2>(*curr))
		{
			*prev = make_tuple(std::get<0>(*prev), std::get<1>(*curr), std::get<2>(*prev));
			curr = intervals.erase(curr);
		}
		else
		{
			prev = curr;
			++curr;
		}
	}
}



int ReservationTable::getHoldingTimeFromSIT(int location)
{
	updateSIT(location);
	if (sit.find(location) == sit.end())
		return 0;
	int t = std::get<1>(sit[location].back());
	if (t < INTERVAL_MAX)
		return INTERVAL_MAX;
	for (auto p = sit[location].rbegin(); p != sit[location].rend(); ++p)
	{
		if (t == std::get<1>(*p))
			t = std::get<0>(*p);
		else
			break;
	}
	return t;
}

int ReservationTable::getHoldingTimeFromCT(int location) const
{
	const auto& it = ct.find(location);
	if (it == ct.end())
		return 0;

	int t = 0;
	for (auto time_range : it->second)
	{
		if (time_range.second > t)
			t = time_range.second;
	}
	return t;
}

set<int> ReservationTable::getConstrainedTimesteps(int location) const
{
    set<int> rst;
    const auto& it = ct.find(location);
    if (it == ct.end())
        return rst;

    for (auto time_range : it->second)
    {
        if (time_range.second == INTERVAL_MAX) // skip goal constraint
            continue;
        for (auto t = time_range.first; t < time_range.second; t++)
            rst.insert(t);
    }
    return rst;
}

void ReservationTable::insertConstraint2SIT(int location, int t_min, int t_max)
{
    if (sit.find(location) == sit.end())
    {
        if (t_min > 0)
        {
			sit[location].emplace_back(0, t_min, 0);
        }
		sit[location].emplace_back(t_max, INTERVAL_MAX, 0);
        return;
    }
    for (auto it = sit[location].begin(); it != sit[location].end();)
    {
        if (t_min >= std::get<1>(*it))
			++it; 
        else if (t_max <= std::get<0>(*it))
            break;
       else  if (std::get<0>(*it) < t_min && std::get<1>(*it) <= t_max)
        {
            (*it) = make_tuple(std::get<0>(*it), t_min, 0);
			++it;
        }
        else if (t_min <= std::get<0>(*it) && t_max < std::get<1>(*it))
        {
            (*it) = make_tuple(t_max, std::get<1>(*it), 0);
            break;
        }
        else if (std::get<0>(*it) < t_min && t_max < std::get<1>(*it))
        {
			sit[location].insert(it, make_tuple(std::get<0>(*it), t_min, 0));
            (*it) = make_tuple(t_max, std::get<1>(*it), 0);
            break;
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            it = sit[location].erase(it);
        }
    }
}

void ReservationTable::insertSoftConstraint2SIT(int location, int t_min, int t_max)
{
    if (sit.find(location) == sit.end())
    {
        if (t_min > 0)
        {
			sit[location].emplace_back(0, t_min, false);
        }
		sit[location].emplace_back(t_min, t_max, true);
		sit[location].emplace_back(t_max, INTERVAL_MAX, false);
        return;
    }
    for (auto it = sit[location].begin(); it != sit[location].end(); it++)
    {
        if (t_min >= std::get<1>(*it))
            continue;
        else if (t_max <= std::get<0>(*it))
            break;
		else if (std::get<2>(*it)) // the interval already has conflicts. No need to update
			continue;

        if (std::get<0>(*it) < t_min && std::get<1>(*it) <= t_max)
        {
			sit[location].insert(it, make_tuple(std::get<0>(*it), t_min, false));
			(*it) = make_tuple(t_min, std::get<1>(*it), true);
        }
        else if (t_min <= std::get<0>(*it) && t_max < std::get<1>(*it))
        {
			sit[location].insert(it, make_tuple(std::get<0>(*it), t_max, true));
            (*it) = make_tuple(t_max, std::get<1>(*it), false);
        }
        else if (std::get<0>(*it) < t_min && t_max < std::get<1>(*it))
        {
			sit[location].insert(it, make_tuple(std::get<0>(*it), t_min, false));
			sit[location].insert(it, make_tuple(t_min, t_max,  true));
            (*it) = make_tuple(t_max, std::get<1>(*it), false);
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            (*it) = make_tuple(std::get<0>(*it), std::get<1>(*it), true);
        }
    }
}


void ReservationTable::insertPath2CT(const Path& path)
{
	if (path.empty())
		return;
	auto prev = path.begin();
	auto curr = path.begin();
	++curr;
	while (curr != path.end() && curr->timestep - k_robust <= window)
	{
		if (prev->location != curr->location)
		{
			if (G.types[prev->location] != "Magic")
				ct[prev->location].emplace_back(prev->timestep - k_robust, curr->timestep + k_robust);
			if (k_robust == 0) // add edge constraint
			{
				ct[getEdgeIndex(curr->location, prev->location)].emplace_back(curr->timestep, curr->timestep + 1);
			}
			prev = curr;
		}
		++curr;
	}
	if (curr != path.end())
	{
		if (G.types[prev->location] != "Magic")
			ct[prev->location].emplace_back(prev->timestep - k_robust, curr->timestep + k_robust);
		if (k_robust == 0) // add edge constraint
		{
			ct[getEdgeIndex(curr->location, prev->location)].emplace_back(curr->timestep, curr->timestep + 1);
		}
	}
	else
	{
		if (G.types[prev->location] != "Magic")
			ct[prev->location].emplace_back(prev->timestep - k_robust, path.back().timestep + 1 + k_robust);
		if (k_robust == 0) // add edge constraint
		{
			ct[getEdgeIndex(path.back().location, prev->location)].emplace_back(path.back().timestep, path.back().timestep + 1);
		}
	}
	if (hold_endpoints && G.types[prev->location] != "Magic")
		ct[path.back().location].emplace_back(path.back().timestep, INTERVAL_MAX);
}

void ReservationTable::addInitialConstraints(const list< tuple<int, int, int> >& initial_constraints, int current_agent)
{
	for (auto con : initial_constraints)
	{
		if (std::get<0>(con) != current_agent && 0 <= std::get<1>(con) && std::get<1>(con) < G.types.size() &&
			G.types[std::get<1>(con)] != "Magic")
			ct[std::get<1>(con)].emplace_back(0, min(window, std::get<2>(con)));
	}
}


//  insert the path to the conflict avoidance table
void ReservationTable::insertPath2CAT(const Path& path)
{
	if (path.empty())
		return;
	int max_timestep = min((int)path.size() - 1, k_robust + window);
	int timestep = 0;
	while (timestep <= max_timestep)
	{
		int location = path[timestep].location;
		if (G.types[location] != "Magic")
		{
			for (int t = max(0, timestep - k_robust); t <= min((int)cat.size() - 1, timestep + k_robust); t++)
			{
				cat[t][location] = true;
			}
		}
		timestep++;
	}
	if (G.types[path.back().location] != "Magic")
	{
		while (timestep < (int)cat.size()) // assume that the agent waits at its last location
		{
			cat[timestep][path.back().location] = true;
			timestep++;
		}
	}
}

// For PBS
void ReservationTable::build(const vector<Path*>& paths,
        const list< tuple<int, int, int> >& initial_constraints,
        const unordered_set<int>& high_priority_agents, int current_agent, int start_location)
{
    clock_t t = std::clock();

    // add hard constraints
    vector<bool> soft(num_of_agents, true);
    for (auto i : high_priority_agents)
    {
        if (paths[i] == nullptr)
            continue;
		insertPath2CT(*paths[i]);
		soft[i] = false;
    }

    if (prioritize_start) // prioritize waits at start locations
    {
        insertConstraints4starts(paths, current_agent, start_location);
    }

	addInitialConstraints(initial_constraints, current_agent); // add initial constraints
   
    runtime = (std::clock() - t) * 1.0  / CLOCKS_PER_SEC;
    if (!use_cat)
        return;

    // add soft constraints
    soft[current_agent] = false;
    for (int i = 0; i < num_of_agents; i++)
    {
        if(!soft[i] || paths[i] == nullptr)
            continue;
		insertPath2CAT(*paths[i]);
    }

    runtime = (std::clock() - t) * 1.0  / CLOCKS_PER_SEC;
}

// For WHCA*
void ReservationTable::build(const vector<Path>& paths,
                            const list< tuple<int, int, int> >& initial_constraints,
                            int current_agent)
{
    clock_t t = std::clock();
    // add hard constraints
    for (int i = 0; i < (int)paths.size(); i++)
    {
		if (i == current_agent)
			continue;
		insertPath2CT(paths[i]);
    }

	addInitialConstraints(initial_constraints, current_agent); // add initial constraints
    runtime = (std::clock() - t) * 1.0  / CLOCKS_PER_SEC;
}

// For ECBS
void ReservationTable::build(const vector<Path*>& paths,
                            const list< tuple<int, int, int> >& initial_constraints,
                            const list< Constraint >& hard_constraints, int current_agent)
{
    clock_t t = std::clock();
    // add hard constraints
    for (auto con : hard_constraints)
    {
        if (std::get<0>(con) == current_agent && std::get<4>(con)) // positive constraint
        {
           // insert_positive_constraint(std::get<1>(con), std::get<3>(con));
		   // TODO: insert positive constraints
        }
		else if (std::get<2>(con) < 0 && G.types[std::get<1>(con)] != "Magic") // vertex constraint
        {
			ct[std::get<1>(con)].emplace_back(std::get<3>(con), std::get<3>(con) + 1);
        }
		else // edge constraint
		{
			ct[getEdgeIndex(std::get<1>(con), std::get<2>(con))].emplace_back(std::get<3>(con), std::get<3>(con) + 1);
		}
    }

	addInitialConstraints(initial_constraints, current_agent); // add initial constraints

    /* add soft constraints */
	// compute the max timestep that cat needs
	size_t cat_size = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        if(i == current_agent || paths[i] == nullptr)
            continue;

		if ((int)paths[i]->size() > window)
		{
			cat_size = window;
			break;
		}
		else if (cat_size < paths[i]->size())
			cat_size = paths[i]->size();
	}
	cat.resize(cat_size, vector<bool>(map_size));

	// build cat
	for (int i = 0; i < num_of_agents; i++)
	{
		if (i == current_agent || paths[i] == nullptr)
			continue;
		
       insertPath2CAT(*paths[i]);
    }
    runtime = (std::clock() - t) * 1.0  / CLOCKS_PER_SEC;
}


void ReservationTable::insertConstraints4starts(const vector<Path*>& paths, int current_agent, int start_location)
{
    for (int i = 0; i < num_of_agents; i++)
    {
        if (paths[i] == nullptr)
            continue;
        else if (i != current_agent)// prohibit the agent from conflicting with other agents at their start locations
        {
            int start = paths[i]->front().location;
            if (start < 0 || G.types[start] == "Magic")
                continue;
            for (auto state : (*paths[i]))
            {
                if (state.location != start) // The agent starts to move
                {
                    // The agent waits at its start locations between [appear_time, state.timestep - 1]
                    // So other agents cannot use this start location between
                    // [appear_time - k_robust, state.timestep + k_robust - 1]
                    ct[start].emplace_back(0, state.timestep + k_robust);
                    break;
                }
            }
        }
    }
}

// [lower_bound, upper_bound)
list<Interval> ReservationTable::getSafeIntervals(int location, int lower_bound, int upper_bound)
{
    list<Interval> safe_intervals;
    if (lower_bound >= upper_bound)
        return safe_intervals;

	updateSIT(location);
	
	auto it = sit.find(location);
    if (it == sit.end()) 
    {
		safe_intervals.emplace_back(0, INTERVAL_MAX, 0);
		return safe_intervals;
    }

    for(auto interval : it->second)
    {
        if (lower_bound >= std::get<1>(interval))
            continue;
        else if (upper_bound <= std::get<0>(interval))
            break;
        else
        {
            safe_intervals.emplace_back(interval);
        }

    }
    return safe_intervals;
}

// [lower_bound, upper_bound)
list<Interval> ReservationTable::getSafeIntervals(int from, int to, int lower_bound, int upper_bound)
{
	if (lower_bound >= upper_bound)
		return list<Interval>();
	
	auto safe_vertex_intervals = getSafeIntervals(to, lower_bound, upper_bound);
	auto safe_edge_intervals = getSafeIntervals(getEdgeIndex(from, to), lower_bound, upper_bound);

	list<Interval> safe_intervals;
	auto it1 = safe_vertex_intervals.begin();
	auto it2 = safe_edge_intervals.begin();
	while (it1 != safe_vertex_intervals.end() && it2 != safe_edge_intervals.end())
	{
		int t_min = max(std::get<0>(*it1), std::get<0>(*it2));
		int t_max = min(std::get<1>(*it1), std::get<1>(*it2));
		if (t_min < t_max)
			safe_intervals.emplace_back(t_min, t_max, std::get<2>(*it1) + std::get<2>(*it2));
		if (t_max == std::get<1>(*it1))
			++it1;
		if (t_max == std::get<1>(*it2))
			++it2;
	}
	return safe_intervals;
}

Interval ReservationTable::getFirstSafeInterval(int location)
{
	updateSIT(location);
    auto it = sit.find(location);
    if (it == sit.end())
    {
		return Interval(0, INTERVAL_MAX, 0);
    }
    return it->second.front();
}

// find a safe interval with t_min as given
bool ReservationTable::findSafeInterval(Interval& interval, int location, int t_min)
{
	updateSIT(location);

    auto it = sit.find(location);
    if (it == sit.end())
    {
		return t_min == 0;
    }
    for( auto i : it->second)
    {
        if (t_min == std::get<0>(i))
        {
            interval = i;
            return true;
        }
        else if (t_min < std::get<0>(i))
            break;
    }
    return false;
}


void ReservationTable::print() const
{
    for (const auto& entry : sit)
    {
        cout << "loc=" << entry.first << ":";
        for (const auto& interval : entry.second)
        {
            cout << "[" << std::get<0>(interval) << "," << std::get<1>(interval) << "],";
        }
    }
    cout << endl;
}

void ReservationTable::printCT(size_t location) const
{
    cout << "loc=" << location << ":";
    const auto it = ct.find(location);
    if (it != ct.end())
    {
        for (const auto & interval : ct.at(location))
        cout << "[" << std::get<0>(interval) << "," << std::get<1>(interval) << "],";
    }
    cout << endl;
}


bool ReservationTable::isConstrained(int curr_id, int next_id, int next_timestep) const
{
	auto it = ct.find(next_id);
	if (it != ct.end())
	{
		for (auto time_range : it->second)
		{
			if (next_timestep >= time_range.first && next_timestep < time_range.second)
				return true;
		}
	}

	if (curr_id != next_id)
	{
		it = ct.find(getEdgeIndex(curr_id, next_id));
		if (it != ct.end())
		{
			for (auto time_range : it->second)
			{
				if (next_timestep >= time_range.first && next_timestep < time_range.second)
					return true;
			}
		}
	}
	return false;
}


bool ReservationTable::isConflicting(int curr_id, int next_id, int next_timestep) const
{
	if (next_timestep >= (int)cat.size())
		return false;

	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (cat[next_timestep][next_id])
		return true;
	// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
	// which means that res_table is occupied with another agent for [curr_id,next_timestep] and [next_id,next_timestep-1]
	// WRONG!
	else if (curr_id != next_id && cat[next_timestep][curr_id] && cat[next_timestep - 1][next_id])
		return true;
	else
		return false;
}