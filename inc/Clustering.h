#pragma once
#include "MDD.h"
#include "node.h"
#include "dataanalysis.h"

using namespace alglib;

//Wooju Yim
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

    void run(); 

    void updateLocations(const std::vector<State>& starts,
              const std::vector< std::vector<std::pair<int, int> > >& goal_locations);

    //Subclusters the 2d array result into a Node tree
    void subcluster(integer_2d_array& arr, int currIndex, Node* headNode);

    void clear() { distances.clear(); }

    Clustering(const BasicGraph& G, const int& planning_window, const int& lookahead):
        G(G), mdd_helper(G, planning_window,lookahead), lookahead(lookahead) {}

    void writeDistanceMatrixToFile();
    std::vector<std::vector<int>>& getClusters() { return clusters; }
private:
    const BasicGraph& G;
    MDDTable mdd_helper;
    const int& lookahead;
    std::vector<std::vector<int>> landmarks;
    std::vector<std::vector<int>> distances;
    std::vector<std::vector<int>> clusters;
    std::vector<MDD*> mdds;

    clock_t start_time;
    // variables and functions for clustering
    int num_of_agents = 0;
    int getDistance(int a1, int a2);
    int getStartVertex(int agent) const { return landmarks[agent][0]; }
    int getFirstGoalVertex(int agent) const { return landmarks[agent][1]; }
    void getAllDistances();


    //Calculate all node's number of leaf chlidren
    //Fills each node with data
    int calcLeafNodesNum(Node* node);

    //Find Nodes that have lower num of leaf children than the limit
    //These nodes will be the head of different subclusters
    void FillClusterNodes(Node* root, std::vector<Node*>& clusteredNodes, int limit);

    //Find all the leaf nodes under this node and add them to the cluster
    void findLeafNodes(Node* node, std::vector<int>& cluster, int limit);

    //Turn the node trees into a List of Lists to subcluster them properly
    void nodesIntoClusters(std::vector<Node*>& clusteredNodes, std::vector<std::vector<int>>& clusters, int limit);

    //Deletes the tree
    void deleteTree(Node* node);

    //In order traversal showing the labels of each node in the graph
    void inorderTraversal(Node* node);

    //Print the results of the clusters
    void print2dvector(std::vector<std::vector<int>>& vectors);

    //Compares two clusters
    //Returns true if clusters are the same
    bool compareClusters(std::vector<std::vector<int>>& vec1, std::vector<std::vector<int>>& vec2);

};

