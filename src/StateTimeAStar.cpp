#include "StateTimeAStar.h"


Path StateTimeAStar::updatePath(const StateTimeAStarNode* goal)
{
    // std::cout << "Update path .. " << std::endl;
    Path path(goal->state.timestep + 1);
    path_cost = goal->getFVal();
    // std::cout << goal->timestep << "," << path.size() <<
    //    "," << goal->conflicts << std::endl;
    num_of_conf = goal->conflicts;
    const StateTimeAStarNode* curr = goal;
    for(int t = goal->state.timestep; t >= 0; t--)
    {
        if (curr->state.timestep > t)
        {
            curr = curr->parent;
            assert (curr->state.timestep <= t);
        }
        path[t] = curr->state;
    }
    return path;
}


list<pair<int, int> > StateTimeAStar::updateTrajectory(const StateTimeAStarNode* goal)
{
    list<pair<int, int> > trajectory;
    path_cost = goal->getFVal();
    const StateTimeAStarNode* curr = goal;
    while (curr != nullptr)
    {
        trajectory.emplace_front(curr->state.location, curr->state.orientation);
        curr = curr->parent;
    }
    return trajectory;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
// after max_timestep, switch from time-space A* search to normal A* search
Path StateTimeAStar::run(const BasicGraph& G, const State& start, 
	const vector<pair<int, int> >& goal_location, ReservationTable& rt)
{
    num_expanded = 0;
    num_generated = 0;
	runtime = 0;
	clock_t t = std::clock();

	double h_val = compute_h_value(G, start.location, 0, goal_location);
	if (h_val > INT_MAX)
	{
		cout << "The start and goal locations are disconnected!" << endl;
		return Path();
	}
    if (rt.isConstrained(start.location, start.location, 0))
        return Path();

	// generate root and add it to the OPEN list
	StateTimeAStarNode* root;
    root = new StateTimeAStarNode(start, 0, h_val, nullptr, 0);
    num_generated++;
    root->open_handle = open_list.push(root);
    root->focal_handle = focal_list.push(root);
    root->in_openlist = true;
    allNodes_table.insert(root);
    min_f_val = root->getFVal();
    double lower_bound = min_f_val;

	int earliest_holding_time = 0;
	if (hold_endpoints)
		earliest_holding_time = rt.getHoldingTimeFromCT(goal_location.back().first);

    while (!focal_list.empty())
    {
        StateTimeAStarNode* curr = focal_list.top(); focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;
		
		// update goal id
        if (curr->state.location == goal_location[curr->goal_id].first && 
			curr->state.timestep >= goal_location[curr->goal_id].second &&
			!(curr->goal_id == (int)goal_location.size() - 1 &&
				earliest_holding_time > curr->state.timestep))
			curr->goal_id++;
		// check if the popped node is a goal
		if (curr->goal_id == (int)goal_location.size())
		{
			Path path = updatePath(curr);
			releaseClosedListNodes();
			open_list.clear();
			focal_list.clear();
			runtime = (double)(std::clock() - t) / CLOCKS_PER_SEC;
			return path;
		}

        for (const auto& next_state: G.get_neighbors(curr->state))
        {
            if (!rt.isConstrained(curr->state.location, next_state.location, next_state.timestep))
            {
                // compute cost to next_id via curr node
                double next_g_val = curr->g_val + G.get_weight(curr->state.location, next_state.location);
                double next_h_val = compute_h_value(G, next_state.location, curr->goal_id, goal_location);
                if (next_h_val >= INT_MAX) // This vertex cannot reach the goal vertex
                    continue;
                int next_conflicts = curr->conflicts;
				if (rt.isConflicting(curr->state.location, next_state.location, next_state.timestep))
					next_conflicts++;

                // generate (maybe temporary) node
                auto next = new StateTimeAStarNode(next_state, next_g_val, next_h_val, curr, next_conflicts);

                // try to retrieve it from the hash table
                auto it = allNodes_table.find(next);
                if (it == allNodes_table.end())
                {
                    next->open_handle = open_list.push(next);
                    next->in_openlist = true;
                    num_generated++;
                    if (next->getFVal() <= lower_bound)
                        next->focal_handle = focal_list.push(next);
                    allNodes_table.insert(next);
                }
                else
                {  // update existing node's if needed (only in the open_list)
                    StateTimeAStarNode* existing_next = *it;

                    if (existing_next->in_openlist)
                    {  // if its in the open list
                        if (existing_next->getFVal() > next_g_val + next_h_val ||
                            (existing_next->getFVal() == next_g_val + next_h_val && existing_next->conflicts > next_conflicts))
                        {
                            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
                            bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
                            bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
                            bool update_open = false;
                            if ((next_g_val + next_h_val) <= lower_bound)
                            {  // if the new f-val qualify to be in FOCAL
                                if (existing_next->getFVal() > lower_bound)
                                    add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                                else
                                    update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
                            }
                            if (existing_next->getFVal() > next_g_val + next_h_val)
                                update_open = true;
                            // update existing node
                            existing_next->g_val = next_g_val;
                            existing_next->h_val = next_h_val;
                            existing_next->parent = curr;
                            existing_next->depth = next->depth;
                            existing_next->conflicts = next_conflicts;
                            // existing_next->move = next->move;

                            if (update_open)
                                open_list.increase(existing_next->open_handle);  // increase because f-val improved
                            if (add_to_focal)
                                existing_next->focal_handle = focal_list.push(existing_next);
                            if (update_in_focal)
                                focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
                        }
                    }
                    else
                    {  // if its in the closed list (reopen)
                        if (existing_next->getFVal() > next_g_val + next_h_val ||
                            (existing_next->getFVal() == next_g_val + next_h_val && existing_next->conflicts > next_conflicts))
                        {
                            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
                            existing_next->g_val = next_g_val;
                            existing_next->h_val = next_h_val;
                            existing_next->parent = curr;
                            existing_next->depth = next->depth;
                            existing_next->conflicts = next_conflicts;
                            existing_next->open_handle = open_list.push(existing_next);
                            existing_next->in_openlist = true;
                            if (existing_next->getFVal() <= lower_bound)
                                existing_next->focal_handle = focal_list.push(existing_next);
                        }
                    }  // end update a node in closed list
                    delete(next);  // not needed anymore -- we already generated it before
                }  // end update an existing node
            }// end if case forthe move is legal
        }  // end for loop that generates successors

        // update FOCAL if min f-val increased
        if (open_list.empty())  // in case OPEN is empty, no path found
        {
            // This is correct only when k_robust <= 1. Otherwise, agents might not be able to
            // wait at its start locations due to initial constraints caused by the previous actions
            // of other agents.
            auto timesteps = rt.getConstrainedTimesteps(start.location);
            auto wait_cost = G.get_weight(start.location, start.location);
            auto h = compute_h_value(G, start.location, 0, goal_location);
            for (int t : timesteps)
            {
                State s(start.location, t, start.orientation);
                auto node2 = new StateTimeAStarNode(s, t * wait_cost, h, root, 0);
                num_generated++;
                node2->open_handle = open_list.push(node2);
                node2->in_openlist = true;
                allNodes_table.insert(node2);
            }
            min_f_val = open_list.top()->getFVal();
            lower_bound = min_f_val;
            open_list.top()->focal_handle = focal_list.push(open_list.top());
        }
        else
        {
            auto open_head = open_list.top();
            assert(!focal_list.empty() or open_head->getFVal() > min_f_val);
            if (open_head->getFVal() > min_f_val)
            {
                double new_min_f_val = open_head->getFVal();
                double new_lower_bound = std::max(lower_bound,  new_min_f_val);
                for (StateTimeAStarNode* n : open_list)
                {
                    if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound)
                        n->focal_handle = focal_list.push(n);
                }
                min_f_val = new_min_f_val;
                lower_bound = new_lower_bound;
            }
            assert(!focal_list.empty());
        }
    }  // end while loop

    // no path found
    releaseClosedListNodes();
    open_list.clear();
    focal_list.clear();
    return Path();
}


void StateTimeAStar::findTrajectory(const BasicGraph& G,
                     const State& start,
                     const vector<pair<int, int> >& goal_locations,
                     const unordered_map<int, double>& travel_times,
                     list<pair<int, int> >& trajectory)
{
    num_expanded = 0;
    num_generated = 0;
    open_list.clear();
    releaseClosedListNodes();

    // generate start and add it to the OPEN list
    double h_val = compute_h_value(G, start.location, 0, goal_locations);
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
            if (curr->goal_id == (int) goal_locations.size())
            {
                trajectory = updateTrajectory(curr);
                releaseClosedListNodes();
                open_list.clear();
                return;
            }
        }

        double travel_time = 1;
        auto p = travel_times.find(curr->state.location);
        if (p != travel_times.end())
        {
            travel_time += p->second;
        }
        for (const auto& next_state: G.get_neighbors(curr->state))
        {
            if (curr->state.location == next_state.location && curr->state.orientation == next_state.orientation)
                continue;
            // compute cost to next_id via curr node
            double next_g_val = curr->g_val + G.get_weight(curr->state.location, next_state.location) * travel_time;
            double next_h_val = compute_h_value(G, next_state.location, curr->goal_id, goal_locations);
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
}


inline void StateTimeAStar::releaseClosedListNodes()
{
    for (auto it : allNodes_table)
        delete it;
    allNodes_table.clear();
}