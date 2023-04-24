#pragma once
#include "ECBSNode.h"
#include "MAPFSolver.h"
#include <ctime>

class ECBS :
	public MAPFSolver
{
public:

    bool disjoint_splitting;

	string potential_function;
	double potential_threshold;
	double suboptimal_bound;


    ECBSNode* dummy_start;

    uint64_t HL_num_expanded;
    uint64_t HL_num_generated;
    uint64_t LL_num_expanded;
    uint64_t LL_num_generated;


    // Runs the algorithm until the problem is solved or time is exhausted
    bool run(const vector<State>& starts,
             const vector< vector<pair<int, int> > >& goal_locations, 
             int time_limit);


    ECBS(const BasicGraph& G, SingleAgentSolver& path_planner);
    ~ECBS();

    void update_paths(ECBSNode* curr);
    void find_conflicts(int start_time, std::list<std::shared_ptr<Conflict> >& conflicts) const;
    // Save results
    void save_results(const std::string &fileName, const std::string &instanceName) const;
    void save_search_tree(const std::string &fileName) const;
	void save_constraints_in_goal_node(const std::string &fileName) const {}

	string get_name() const {return "ECBS"; }
    void clear();

private:
    typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::compare_node> > heap_open_t;
    typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::secondary_compare_node> > heap_focal_t;
    heap_open_t open_list;
    heap_focal_t focal_list;
    list<ECBSNode*> allNodes_table;

    ECBSNode* best_node;

    // vector<State> starts;
    // vector< vector<int> > goal_locations;
    std::vector< Path* > paths;
    std::vector<double> path_min_costs;
    std::vector<double> path_costs;

    std::clock_t start;

    double min_f_val;
    double focal_threshold;

    // double focal_w = 1.0;
    // unordered_map<int, double> travel_times;

    unordered_set<pair<int, int>> nogood;

    bool generate_root_node();
    void push_node(ECBSNode* node);
	void reinsert_node(ECBSNode* node);
    ECBSNode* pop_node();

    // high level search
    bool find_path(ECBSNode*  node, int ag);
    void resolve_conflict(const Conflict& conflict, ECBSNode* n1, ECBSNode* n2);
    bool generate_child(ECBSNode* child, ECBSNode* curr);

    // conflicts
    void remove_conflicts(std::list<std::shared_ptr<Conflict> >& conflicts, int excluded_agent) const;
    void find_conflicts(const std::list<std::shared_ptr<Conflict> >& old_conflicts,
                        std::list<std::shared_ptr<Conflict> >& new_conflicts, const list<int>& new_agents) const;
    void find_conflicts(int start_time, std::list<std::shared_ptr<Conflict> >& conflicts, int a1, int a2) const;
    void find_conflicts(std::list<std::shared_ptr<Conflict> >& new_conflicts, int new_agent) const;
    void choose_conflict(ECBSNode &parent) const;
    void copy_conflicts(const std::list<std::shared_ptr<Conflict> >& conflicts,
                        std::list<std::shared_ptr<Conflict> >& copy, const list<int>& new_agents) const;

    //double get_path_cost(const Path& path) const;

    //update information
    //void collect_constraints(ECBSNode* curr, int current_agent);

    //void update_CAT(int ex_ag); // update conflict avoidance table
    void update_focal_list();
    inline void release_closed_list();

    // print and save
    void print_paths() const;
    void print_results() const;
    void print_conflicts(const ECBSNode &curr) const;

    void get_solution();

    // validate
    bool validate_solution() const;
    bool validate_path(const Path& path, const list<Constraint>& constraints) const;

};
