#include "BasicGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>


void BasicGraph::print_map() const
{  
    std::cout << "***type***" << std::endl;
    for (std::string t : types)
        std::cout << t << ",";
    std::cout << std::endl;

    std::cout << "***weights***" << std::endl;
    for (std::vector<double> n : weights)
    {
        for (double w : n)
        {
            std::cout << w << ",";
        }
        std::cout << std::endl;
    }
}


int BasicGraph::get_rotate_degree(int dir1, int dir2) const
{
    if (dir1 == dir2)
        return 0;
    else if (abs(dir1 - dir2) == 1 || abs(dir1 - dir2) == 3)
        return 1;
    else
        return 2;
}


list<int> BasicGraph::get_neighbors(int v) const
{
    list<int> neighbors;
    if (v < 0)
        return neighbors;

    for (int i = 0; i < 4; i++) // move
        if (weights[v][i] < WEIGHT_MAX - 1)
            neighbors.push_back(v + move[i]);

    return neighbors;
}

list<State> BasicGraph::get_neighbors(const State& s) const
{
    list<State> neighbors;
    if (s.location < 0)
        return neighbors;
    if (s.orientation >= 0)
    {
        neighbors.push_back(State(s.location, s.timestep + 1, s.orientation)); // wait
        if (weights[s.location][s.orientation] < WEIGHT_MAX - 1)
            neighbors.push_back(State(s.location + move[s.orientation], s.timestep + 1, s.orientation)); // move
        int next_orientation1 = s.orientation + 1;
        int next_orientation2 = s.orientation - 1;
        if (next_orientation2 < 0)
            next_orientation2 += 4;
        else if (next_orientation1 > 3)
            next_orientation1 -= 4;
        neighbors.push_back(State(s.location, s.timestep + 1, next_orientation1)); // turn left
        neighbors.push_back(State(s.location, s.timestep + 1, next_orientation2)); // turn right
    }
    else
    {
        neighbors.push_back(State(s.location, s.timestep + 1)); // wait
        for (int i = 0; i < 4; i++) // move
            if (weights[s.location][i] < WEIGHT_MAX - 1)
                neighbors.push_back(State(s.location + move[i], s.timestep + 1));
    }
    return neighbors;
}    

std::list<State> BasicGraph::get_reverse_neighbors(const State& s) const
{
    std::list<State> rneighbors;
    // no wait actions
    if (s.orientation >= 0)
    {
        if (s.location - move[s.orientation] >= 0 && s.location - move[s.orientation] < this->size() &&
            weights[s.location - move[s.orientation]][s.orientation] < WEIGHT_MAX - 1)
            rneighbors.push_back(State(s.location - move[s.orientation], -1, s.orientation)); // move
        int next_orientation1 = s.orientation + 1;
        int next_orientation2 = s.orientation - 1;
        if (next_orientation2 < 0)
            next_orientation2 += 4;
        else if (next_orientation1 > 3)
            next_orientation1 -= 4;
        rneighbors.push_back(State(s.location, -1, next_orientation1)); // turn right
        rneighbors.push_back(State(s.location, -1, next_orientation2)); // turn left
    }
    else
    {
        for (int i = 0; i < 4; i++) // move
            if (s.location - move[i] >= 0 && s.location - move[i] < this->size() &&
                    weights[s.location - move[i]][i] < WEIGHT_MAX - 1)
                rneighbors.push_back(State(s.location - move[i]));
    }
    return rneighbors;
}


double BasicGraph::get_weight(int from, int to) const
{
    if (from == to) // wait or rotate
        return weights[from][4];
    int dir = get_direction(from, to);
    if (dir >= 0)
        return weights[from][dir];
    else
        return WEIGHT_MAX;
}


int BasicGraph::get_direction(int from, int to) const
{
    for (int i = 0; i < 4; i++)
    {
        if (move[i] == to - from)
            return i;
    }
    if (from == to)
        return 4;
    return -1;
}



bool BasicGraph::load_heuristics_table(std::ifstream& myfile)
{
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> >::iterator beg;
    std::string line;
    
    getline(myfile, line); //skip "table_size"
    getline(myfile, line);
    boost::tokenizer< boost::char_separator<char> > tok(line, sep);
    beg = tok.begin();
	int N = atoi ( (*beg).c_str() ); // read number of cols
	beg++;
	int M = atoi ( (*beg).c_str() ); // read number of rows
	if (M != this->size())
	    return false;
	for (int i = 0; i < N; i++)
	{
		getline (myfile, line);
        int loc = atoi(line.c_str());
        getline (myfile, line);        
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	    beg = tok.begin();
        std::vector<double> h_table(this->size());
        for (int j = 0; j < this->size(); j++)
        {
            h_table[j] = atof((*beg).c_str());
            if (h_table[j] >= INT_MAX && types[j] != "Obstacle")
                types[j] = "Obstacle";
            beg++;
        }
        heuristics[loc] = h_table;
    }
	return true;
}


void BasicGraph::save_heuristics_table(std::string fname)
{
    std::ofstream myfile;
	myfile.open (fname);
	myfile << "table_size" << std::endl << 
        heuristics.size() << "," << this->size() << std::endl;
	for (auto h_values: heuristics) 
	{
        myfile << h_values.first << std::endl;
		for (double h : h_values.second) 
		{
            myfile << h << ",";
		}
		myfile << std::endl;
	}
	myfile.close();
}

std::vector<double> BasicGraph::compute_heuristics(int root_location)
{
    std::vector<double> res(this->size(), DBL_MAX);
	fibonacci_heap< StateTimeAStarNode*, compare<StateTimeAStarNode::compare_node> > heap;
    unordered_set< StateTimeAStarNode*, StateTimeAStarNode::Hasher, StateTimeAStarNode::EqNode> nodes;

    State root_state(root_location);
    if(consider_rotation)
    {
        for (auto neighbor : get_reverse_neighbors(root_state))
        {
            StateTimeAStarNode* root = new StateTimeAStarNode(State(root_location, -1,
                    get_direction(neighbor.location, root_state.location)), 0, 0, nullptr, 0);
            root->open_handle = heap.push(root);  // add root to heap
            nodes.insert(root);       // add root to hash_table (nodes)
        }
    }
    else
    {
        StateTimeAStarNode* root = new StateTimeAStarNode(root_state, 0, 0, nullptr, 0);
        root->open_handle = heap.push(root);  // add root to heap
        nodes.insert(root);       // add root to hash_table (nodes)
    }

	while (!heap.empty()) 
    {
        StateTimeAStarNode* curr = heap.top();
		heap.pop();
		for (auto next_state : get_reverse_neighbors(curr->state))
		{
			double next_g_val = curr->g_val + get_weight(next_state.location, curr->state.location);
            StateTimeAStarNode* next = new StateTimeAStarNode(next_state, next_g_val, 0, nullptr, 0);
			auto it = nodes.find(next);
			if (it == nodes.end()) 
			{  // add the newly generated node to heap and hash table
				next->open_handle = heap.push(next);
				nodes.insert(next);
			}
			else 
			{  // update existing node's g_val if needed (only in the heap)
				delete(next);  // not needed anymore -- we already generated it before
                StateTimeAStarNode* existing_next = *it;
				if (existing_next->g_val > next_g_val) 
				{
					existing_next->g_val = next_g_val;
					heap.increase(existing_next->open_handle);
				}
			}
		}
	}
	// iterate over all nodes and populate the distances
	for (auto it = nodes.begin(); it != nodes.end(); it++)
	{
        StateTimeAStarNode* s = *it;
		res[s->state.location] = std::min(s->g_val, res[s->state.location]);
		delete (s);
	}
	nodes.clear();
	heap.clear();
    return res;
}


int BasicGraph::get_Manhattan_distance(int loc1, int loc2) const
{
    return abs(loc1 / cols - loc2 / cols) + abs(loc1 % cols - loc2 % cols);
}


void BasicGraph::copy(const BasicGraph& copy)
{
    rows = copy.get_rows();
    cols = copy.get_cols();
    weights = copy.get_weights();
}