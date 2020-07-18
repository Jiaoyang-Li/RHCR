#pragma once
#include "BasicGraph.h"

#define MAX_NUM_OF_MDDS 100000

//TODO: the current MDD does not work with orientation nor with edge-weighted graph
class MDDNode
{
public:
	MDDNode(int location, MDDNode* parent=nullptr): location(location)
	{
		if(parent == nullptr) // root node
        {
            level = 0;
            goal_id = 1;
        }
		else
		{
			level = parent->level + 1;
            goal_id = parent->goal_id;
			parents.push_back(parent);
		}
	}
	int location;
	int level;
    int cost = 0; // minimum additional cost of path traversing this MDD node
    int goal_id;

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}

	//list<MDDNode*> children;
	list<MDDNode*> parents;
};

class MDD
{
public:
	vector<list<MDDNode>> levels;
    const MDDNode* find(int location, int level) const;

	MDD()= default;
    explicit MDD(int num_of_levels) { levels.resize(num_of_levels); }
};

std::ostream& operator<<(std::ostream& os, const MDD& mdd);


class MDDTable
{
public:
	double accumulated_runtime = 0;  // runtime of building MDDs
	uint64_t num_of_mdds = 0; // number of MDDs that have been built
    MDD* getMDD(const vector<int>& landmarks); // landmarks = [start loc, goal loc1, goal loc2, ...]
    void clear();

	MDDTable(const BasicGraph& G, const int& planning_window, const int& lookahead):
	    G(G), planning_window(planning_window), lookahead(lookahead) {}
	~MDDTable() { clear(); }
private:
    const BasicGraph& G;
    const int& planning_window;
    const int& lookahead;
	unordered_map<vector<int>, MDD*> lookupTable;
	void releaseMDDMemory();
    MDD* buildMDD(const vector<int>& landmarks);
};

