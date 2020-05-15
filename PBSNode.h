#pragma once
#include "common.h"
#include "PriorityGraph.h"
#include "States.h"

class PBSNode
{
public:
	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const PBSNode* n1, const PBSNode* n2) const
		{
            if (n1->f_val == n2->f_val)
            {
                return n1->num_of_collisions >= n2->num_of_collisions;
            }
			return n1->f_val >= n2->f_val;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

	// the following is used to comapre nodes in the FOCAL list
	// struct secondary_compare_node
	// {
	// 	bool operator()(const PBSNode* n1, const PBSNode* n2) const
	// 	{
			/*if (n1->num_of_collisions == n2->num_of_collisions)
			{
				if(rand()%2 == 0)
					return true;
				else
					return false;
			}*/
	// 		return n1->num_of_collisions >= n2->num_of_collisions;
	// 	}
	// };  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	typedef fibonacci_heap< PBSNode*, compare<PBSNode::compare_node> >::
	    handle_type open_handle_t;
	// typedef fibonacci_heap< PBSNode*, compare<PBSNode::secondary_compare_node> >::
	//     handle_type focal_handle_t;
	// open_handle_t open_handle;
	// focal_handle_t focal_handle;


	// conflicts in the current paths
	std::list<Conflict> conflicts;
	
	// The chosen conflict
	Conflict conflict;

	PBSNode* parent;


    list< pair<int, Path> > paths; // <agent_id, path>
    std::pair<int, int> priority; // a1 < a2

    PriorityGraph priorities;

	double g_val;
	double h_val;
	double f_val;
	size_t depth; // depath of this CT node
	size_t makespan; // makespan over all paths
	int num_of_collisions; // number of conflicts in the current paths
    int earliest_collision;

	uint64_t time_expanded;
	uint64_t time_generated;


    void print_priorities() const;

	void clear();

	PBSNode(): parent(nullptr), g_val(0), h_val(0), earliest_collision(INT_MAX), time_expanded(0) {}
	~PBSNode(){};


private:
};

