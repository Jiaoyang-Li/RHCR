#include "SIPP.h"


Path SIPP::updatePath(const BasicGraph& G, const SIPPNode* goal)
{
    Path path(goal->state.timestep + 1);
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    const SIPPNode* curr = goal;
    while (true)
    {
        if (curr->parent == nullptr) // root node
        {
            int t = curr->state.timestep;
            path[t] = curr->state;
            t--;
            for (; t >= 0; t--)
            {
                path[t] = State(-1, -1); // dummy start states
            }
            break;
        }
        else
        {
            const SIPPNode* prev = curr->parent;
            int degree = G.get_rotate_degree(prev->state.orientation, curr->state.orientation);
            int t = prev->state.timestep + 1;
            if (degree == 1) // turn right or turn left
            {
                path[t] = State(prev->state.location, t, curr->state.orientation);
                t++;
            }
            else if (degree == 2) // turn back
            {
                path[t] = State(prev->state.location, t, (prev->state.orientation + 1) % 4); // turn right
                t++;
                path[t] = State(prev->state.location, t, curr->state.orientation); // turn right
                t++;
            }
            while ( t < curr->state.timestep)
            {
                path[t] = State(prev->state.location, t, curr->state.orientation); // wait at prev location
                t++;
            }
            path[curr->state.timestep] = State(curr->state.location, curr->state.timestep, curr->state.orientation); // move to current location
            curr = prev;
        }
    }
    return path;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
// after max_timestep, switch from time-space A* search to normal A* search
Path SIPP::run(const BasicGraph& G, const State& start,
               const vector<pair<int, int> >& goal_location,
               ReservationTable& rt)
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
    Interval interval = rt.getFirstSafeInterval(start.location);
	
    if (std::get<0>(interval) == 0)
    {
        auto node = new SIPPNode(start, 0, h_val, interval, nullptr, 0);
        num_generated++;
        node->open_handle = open_list.push(node);
        node->in_openlist = true;
        allNodes_table.insert(node);
        min_f_val = node->getFVal();
        focal_bound = min_f_val * suboptimal_bound;
        node->focal_handle = focal_list.push(node);
    }
    else if(prioritize_start) // the agent has the highest priority at its start location
    {
        // This is correct only when k_robust <= 1. Otherwise, agents might not be able to
        // wait at its start locations due to initial constraints caused by the previous actions
        // of other agents.
        Interval interval = make_tuple(0, INTERVAL_MAX, 0);
        auto node = new SIPPNode(start, 0, h_val, interval, nullptr, 0);
        num_generated++;
        node->open_handle = open_list.push(node);
        node->in_openlist = true;
        allNodes_table.insert(node);
        min_f_val = node->getFVal();
        focal_bound = min_f_val;
        node->focal_handle = focal_list.push(node);
    }
	int earliest_holding_time = 0;
	if (hold_endpoints)
		earliest_holding_time = rt.getHoldingTimeFromSIT(goal_location.back().first);
    while (!focal_list.empty())
    {
        SIPPNode* curr = focal_list.top(); focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

         // update goal id
        if (curr->state.location == goal_location[curr->goal_id].first &&
			curr->state.timestep >= goal_location[curr->goal_id].second) // reach the goal location after its release time
        {
            curr->goal_id++;
			if (curr->goal_id == (int)goal_location.size() &&
				earliest_holding_time > curr->state.timestep)
				curr->goal_id--;
        }
		// check if the popped node is a goal
		if (curr->goal_id == (int)goal_location.size())
		{
			Path path = updatePath(G, curr);
			releaseClosedListNodes();
			open_list.clear();
			focal_list.clear();
			runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
			return path;
		}


        // expand the nodes
        for (int orientation = 0; orientation < 4; orientation++) // move
        {
            if (!G.valid_move(curr->state.location, orientation)) // the edge is blocked
                continue;
            int degree;
            if (curr->state.orientation < 0)
                degree = 0;
            else
                degree = G.get_rotate_degree(curr->state.orientation, orientation);
            if (degree > std::get<1>(curr->interval) - curr->state.timestep) // don't have enough time to turn
                continue;
            int location = curr->state.location + G.move[orientation];
            double h_val = compute_h_value(G, location, curr->goal_id, goal_location);
            if (h_val > INT_MAX)   // This vertex cannot reach the goal vertex
                continue;
            int min_timestep = curr->state.timestep + degree + 1;
            for (auto interval : rt.getSafeIntervals(curr->state.location, location, min_timestep, std::get<1>(curr->interval) + 1))
            {
                if (curr->state.orientation < 0)
                    generate_node(interval, curr, G, location, min_timestep, -1, h_val);
                else
                    generate_node(interval, curr, G, location, min_timestep, orientation, h_val);
            }

        }  // end for loop that generates successors

        if(rt.use_cat) // wait to the successive interval
        {
            int location = curr->state.location;
            int min_timestep = std::get<1>(curr->interval);
            int orientation = curr->state.orientation;
            Interval interval;
            bool found = rt.findSafeInterval(interval, location, min_timestep);
            if (found)
            {
				if (curr->state.orientation < 0)
				{
					generate_node(interval, curr, G, location, min_timestep, -1, curr->h_val);
				}
				else
				{
					generate_node(interval, curr, G, location, min_timestep, orientation, curr->h_val);
					generate_node(interval, curr, G, location, min_timestep, (orientation + 1) % 4, curr->h_val);
					generate_node(interval, curr, G, location, min_timestep, (orientation + 3) % 4, curr->h_val);
					if (std::get<1>(curr->interval) - curr->state.timestep > 1)
						generate_node(interval, curr, G, location, min_timestep, (orientation + 2) % 4, curr->h_val);
				}
            }
        }

        // update FOCAL if min f-val increased
        if (open_list.empty())  // in case OPEN is empty, no path found
        {
            if(prioritize_start) // the agent has the highest priority at its start location
            {
                // This is correct only when k_robust <= 1. Otherwise, agents might not be able to
                // wait at its start locations due to initial constraints caused by the previous actions
                // of other agents.
                Interval interval = rt.getFirstSafeInterval(start.location);
                Interval interval2 = make_tuple(std::get<1>(interval), INTERVAL_MAX, 0);
                double h_val = compute_h_value(G, start.location, 0, goal_location);
                auto node2 = new SIPPNode(start, 0, h_val, interval2, nullptr, 0);
                num_generated++;
                node2->open_handle = open_list.push(node2);
                node2->in_openlist = true;
                allNodes_table.insert(node2);
                min_f_val = node2->getFVal();
                focal_bound = min_f_val;
                node2->focal_handle = focal_list.push(node2);
            }
            else
            {
                break;
            }
        }
        else
        {
            SIPPNode* open_head = open_list.top();
            if (open_head->getFVal() > min_f_val)
            {
                double new_min_f_val = open_head->getFVal();
                double new_focal_bound = new_min_f_val * suboptimal_bound;
                for (SIPPNode* n : open_list)
                {
                    if (n->getFVal() > focal_bound && n->getFVal() <= new_focal_bound)
                        n->focal_handle = focal_list.push(n);
                }
                min_f_val = new_min_f_val;
                focal_bound = new_focal_bound;
            }
        }

    }  // end while loop

    // no path found
    releaseClosedListNodes();
    open_list.clear();
    focal_list.clear();
    return Path();
}


/*void SIPP::generate_node(SIPPNode* curr, const SortationGrid& G,
                         int location, int timestep, int orientation, double h_val)
{
    int wait_time = timestep - curr->state.timestep - 1; // inlcude rotate time
    double travel_time = 1;
    if (!travel_times.empty())
    {
        int dir = G.get_direction(curr->state.location, location);
        travel_time += travel_times[curr->state.location][dir];
    }
    double g_val = curr->g_val + travel_time * (wait_time * G.get_weight(curr->state.location, curr->state.location)
                   + G.get_weight(curr->state.location, location));

    int conflicts = curr->conflicts;

    // generate (maybe temporary) node
    auto next = new SIPPNode(State(location, timestep, orientation),
                             g_val, h_val, Interval(window + 1, INTERVAL_MAX, 0), curr, conflicts);

    // try to retrieve it from the hash table
    auto it = allNodes_table.find(next);
    if (it != allNodes_table.end() && (*it)->state.timestep != next->state.timestep)
    { // arrive at the same interval at different timestep
        int waiting_time = (*it)->state.timestep - next->state.timestep;
        double waiting_cost = abs(G.get_weight(next->state.location, next->state.location) * waiting_time);
        double next_f_val = next->getFVal() + waiting_cost;
        if (waiting_time > 0 && next_f_val <= (*it)->getFVal())
        { // next arrives earlier with a smaller cost
            // so delete it
            // let the following update it with next
        }
        else if (waiting_time < 0 && next_f_val >= (*it)->getFVal())
        { // it arrives earlier with a smaller cost
            delete next; // so delete next
            return;
        }
        else // the later node arrives with a smaller cost, so they cannot be regarded as the same state
            it = allNodes_table.end();
    }
    if (it == allNodes_table.end())
    {
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // update existing node if needed (only in the open_list)
    SIPPNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();

    if (existing_next->in_openlist)
    {  // if its in the open list
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts > conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
            bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
            bool update_open = false;
            if ((g_val + h_val) <= focal_bound)
            {  // if the new f-val qualify to be in FOCAL
                if (existing_f_val > focal_bound)
                    add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                else
                    update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
            }
            if (existing_f_val > g_val + h_val)
                update_open = true;
            // update existing node
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
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
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts > conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  // end update a node in closed list

    delete(next);  // not needed anymore -- we already generated it before
}*/


void SIPP::generate_node(const Interval& interval, SIPPNode* curr, const BasicGraph& G,
        int location, int min_timestep, int orientation, double h_val)
{
    int timestep  = max(std::get<0>(interval), min_timestep);
    int wait_time = timestep - curr->state.timestep - 1; // inlcude rotate time
    double g_val = curr->g_val + wait_time * G.get_weight(curr->state.location, curr->state.location)
                   + G.get_weight(curr->state.location, location);

    int conflicts = std::get<2>(interval) + curr->conflicts;

    // generate (maybe temporary) node
    auto next = new SIPPNode(State(location, timestep, orientation),
                             g_val, h_val, interval, curr, conflicts);

    // try to retrieve it from the hash table
    auto it = allNodes_table.find(next);
    /*if (it != allNodes_table.end() && (*it)->state.timestep != next->state.timestep)
    { // arrive at the same interval at different timestep
        int waiting_time = (*it)->state.timestep - next->state.timestep;
        double waiting_cost = abs(G.get_weight(next->state.location, next->state.location) * waiting_time);
        if (waiting_time > 0 && next->getFVal() + waiting_cost <= (*it)->getFVal())
        { // next arrives later with a smaller cost
            // let the following update it with next
        }
        else if (waiting_time < 0 && next->getFVal() >= (*it)->getFVal() + waiting_cost)
        { // next arrives earlier with a larger cost
            delete next; // so delete next
            return;
        }
        else // next arrives with a smaller cost, so they cannot be regarded as the same state
            it = allNodes_table.end(); // TODO: fix this bug! When later inserting this node to allNodes_table, it will not override the previous node.
    }*/
    if (it == allNodes_table.end())
    {
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // update existing node if needed (only in the open_list)
    SIPPNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();

    if (existing_next->in_openlist)
    {  // if its in the open list
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts > conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
            bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
            bool update_open = false;
            if ((g_val + h_val) <= focal_bound)
            {  // if the new f-val qualify to be in FOCAL
                if (existing_f_val > focal_bound)
                    add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                else
                    update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
            }
            if (existing_f_val > g_val + h_val)
                update_open = true;
            // update existing node
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
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
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts > conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  // end update a node in closed list

    delete(next);  // not needed anymore -- we already generated it before
}

inline void SIPP::releaseClosedListNodes()
{
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
        delete (*it);
    allNodes_table.clear();
}




