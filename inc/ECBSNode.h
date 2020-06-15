#pragma once
#include "common.h"
#include "States.h"

class ECBSNode
{
public:
    // the following is used to comapre nodes in the OPEN list
    struct compare_node
    {
        bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
        {
            return n1->min_f_val >= n2->min_f_val;
        }
    };  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

    // the following is used to comapre nodes in the FOCAL list
    struct secondary_compare_node
    {
        bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
     	{
            if (n1->num_of_collisions == n2->num_of_collisions)
            {
                return n1->f_val >= n2->f_val;
            }
     		return n1->num_of_collisions >= n2->num_of_collisions;
     	}
    };  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

    typedef fibonacci_heap< ECBSNode*, compare<ECBSNode::compare_node> >::
        handle_type open_handle_t;
    typedef fibonacci_heap< ECBSNode*, compare<ECBSNode::secondary_compare_node> >::
        handle_type focal_handle_t;
    open_handle_t open_handle;
    focal_handle_t focal_handle;
    bool in_openlist;

    // conflicts in the current paths
    std::list<std::shared_ptr<Conflict> > conflicts;
	// int window; // conflicts are detected only before this window

    // The chosen conflict
    std::shared_ptr<Conflict> conflict;

    ECBSNode* parent;


    list< tuple<int, Path, double, double> > paths; // <agent_id, path, lower_bound, path_cost>
    std::list<Constraint> constraints; // constraints imposed to agent_id

    double g_val;
    double h_val;
    double f_val;
    double min_f_val;
    size_t depth; // depth of this CT node
    int num_of_collisions; // number of conflicts in the current paths
    uint64_t time_expanded;
    uint64_t time_generated;

	int window;

    void clear();

    ECBSNode(): parent(nullptr), g_val(0), h_val(0), min_f_val(0), time_expanded(0), time_generated(0) {}
    ECBSNode(ECBSNode* parent);
    ~ECBSNode(){};
};