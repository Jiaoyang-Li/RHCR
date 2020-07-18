#pragma once
#include "MDD.h"

class Clustering {
public:
    // linkage type:
    // -1 uniformly at random
    // 0 complete linkage
    // 1 single linkage
    // 2 unweighted average linkage
    // 3 weighted average linkage
    // 4 Ward's method
    int linkage_type = -1;

    void run(); //Wooju

    void updateLocations(const vector<State>& starts,
              const vector< vector<pair<int, int> > >& goal_locations);

    void clear() { distances.clear(); }


    Clustering(const BasicGraph& G, const int& planning_window, const int& lookahead):
        G(G), mdd_helper(G, planning_window,lookahead), lookahead(lookahead) {}

    void writeDistanceMatrixToFile();
    std::vector<vector<int>>& getClusters() { return clusters; }
private:
    const BasicGraph& G;
    MDDTable mdd_helper;
    const int& lookahead;
    std::vector<vector<int>> landmarks;
    std::vector<vector<int>> distances;
    std::vector<vector<int>> clusters;
    std::vector<MDD*> mdds;

    clock_t start_time;
    // variables and functions for clustering
    int num_of_agents = 0;
    int getDistance(int a1, int a2);
    int getStartVertex(int agent) const { return landmarks[agent][0]; }
    int getFirstGoalVertex(int agent) const { return landmarks[agent][1]; }
    void getAllDistances();

};

