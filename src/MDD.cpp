#include "MDD.h"
#include "SingleAgentSolver.h"

// Build an MDD for the first [planning_window] levels,
// subject to that, for every MDD node n,
// the minimum length of a path via n that visits all landmarks is <= upper_bound
MDD* MDDTable::buildMDD(const vector<int>& landmarks)
{
    int lower_bound = SingleAgentSolver::compute_h_value(G, landmarks[0], 1, landmarks);
    int upper_bound = lookahead + lower_bound;
    auto mdd = new MDD(planning_window + 1);
    mdd->levels[0].emplace_back(landmarks[0]);
    mdd->levels[0].back().cost = 0;
	std::queue<MDDNode*> open;
	open.push(&mdd->levels[0].back());
	while (!open.empty())
	{
		auto curr = open.front();
		open.pop();
		if (curr->level == planning_window)
			continue;
		if (curr->goal_id == (int)landmarks.size())
        {
            mdd->levels[curr->level + 1].emplace_back(curr->location, curr);
            auto next = &mdd->levels[curr->level + 1].back();
            next->cost = curr->cost + 1;
            open.push(next);
            continue;
        }
		// We want (g + 1) + h <= numOfLevels, so h <= numOfLevels - g - 1.
		int heuristicBound = upper_bound - curr->level - 1;
		for (auto next_location : G.get_neighbors(curr->location)) // Try every possible move. We only add backward edges in this step.
		{
		    int h = SingleAgentSolver::compute_h_value(G, next_location, curr->goal_id, landmarks);
			if (h > heuristicBound)
			    continue;
            bool find = false;
            for (auto& child: mdd->levels[curr->level + 1])
            {
                if (child.location == next_location) // If the child node exists
                {
                    child.parents.push_back(curr); // then add corresponding parent link and child link
                    find = true;
                    break;
                }
            }
            if (!find) // Else generate a new mdd node
            {
                mdd->levels[curr->level + 1].emplace_back(next_location, curr);
                auto next = &mdd->levels[curr->level + 1].back();
                next->cost = next->level + h - lower_bound;
                if (next->location == landmarks[next->goal_id])
                    next->goal_id++;
                open.push(next);
            }
		}
	}
	return mdd;
}


const MDDNode* MDD::find(int location, int level) const
{
	if(level < (int)levels.size())
		for (auto& it : levels[level])
			if(it.location == location)
				return &it;
	return nullptr;
}

std::ostream& operator<<(std::ostream& os, const MDD& mdd)
{
	for (const auto& level : mdd.levels)
	{
		cout << "L" << level.front().level << ": ";
		for (const auto& node : level)
		{
			cout << node.location << ",";
		}
		cout << endl;
	}
	return os;
}

MDD * MDDTable::getMDD(const vector<int>& landmarks)
{
    assert(landmarks.size() >= 2);
	auto got = lookupTable.find(landmarks);
	if (got != lookupTable.end())
		return got->second;
	releaseMDDMemory();
	clock_t t = clock();
	auto mdd = buildMDD(landmarks);
	lookupTable[landmarks] = mdd;
    num_of_mdds++;
	accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;
	return mdd;
}

void MDDTable::releaseMDDMemory()
{
	if ((int)lookupTable.size() > MAX_NUM_OF_MDDS)
        clear();
}

void MDDTable::clear()
{
	for (auto& mdd : lookupTable)
	{
		delete mdd.second;
	}
	lookupTable.clear();
}


