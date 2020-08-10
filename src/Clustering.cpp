#include "Clustering.h"
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "dataanalysis.h"
#include "node.h"
#include <queue>

using namespace alglib;

void Clustering::run() //Wooju
{
    //Clear clusters
    clusters.clear();

    if (linkage_type == -1)
    {
        clusters.resize(2);
        for (int i = 0; i < num_of_agents; i++)
        {
            int label = rand() % 2;
            clusters[label].push_back(i);
        }
    }
    else
    {
        //Update distance matrix
        getAllDistances();
        real_2d_array xy;
        xy.setlength(num_of_agents, num_of_agents);
        for (int i = 0; i < num_of_agents; i++) {
            for (int j = 0; j < num_of_agents; j++) {
                xy(i, j) = distances[i][j];
            }
        }
        clusterizerstate clusterstate;
        ahcreport report;
        integer_1d_array cidx;
        integer_1d_array cz;
        clusterizercreate(clusterstate);
        //Upper Triangle
        clusterizersetdistances(clusterstate, xy, true);
        clusterizersetahcalgo(clusterstate, linkage_type);
        //5 is the number of clusters
        clusterizerrunahc(clusterstate, report);

        clusterizergetkclusters(report, 2, cidx, cz);
        //printf("%s\n", cidx.tostring().c_str());
        //printf("%s\n", report.z.tostring().c_str());

        Node* root = new Node(-1, nullptr, nullptr, nullptr);

        subcluster(report.z, report.z.rows() - 1, root);

        inorderTraversal(root);
        deleteTree(root);





        vector<int> cluster1, cluster2;
        for (int index = 0; index < cidx.length(); ++index) {
            //First cluster
            if (cidx[index] == 0) {
                cluster1.emplace_back(index);
            }
                //Second cluster
            else if(cidx[index] == 1){
                cluster2.emplace_back(index);
            }
        }
        clusters.push_back(cluster1);
        clusters.push_back(cluster2);
    }
}

void Clustering::writeDistanceMatrixToFile()
{
    string path = "../python-tools/";
    std::ofstream output;
    output.open(path + "distances.csv", std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        for (int j = 0; j < num_of_agents - 1; j++)
        {
            output << getDistance(i, j) << ",";
        }
        output << getDistance(i, num_of_agents - 1) << endl;
    }
    output.close();
    output.open(path + "agents.csv", std::ios::out);
    output << "start,goal" << endl;
    for (int i = 0; i < num_of_agents; i++)
    {
        output << getStartVertex(i) << "," << getFirstGoalVertex(i) << endl;
    }
    output.close();
    cout << (double)(clock() - start_time) / CLOCKS_PER_SEC << "s" << endl;
}

//Use a BFS iterative method
// 1. Input current [int, int] into queue(FIFO)
// 2. Pop and then input child nodes right first, left second
// 3. Continue until queue is empty and arr is empty as well
void Clustering :: subcluster(integer_2d_array& arr, int currIndex, Node* headNode)
{
    std::queue<ae_int_t*> qe;
    std::queue<Node*> listofNodes;
    const int numofRows = arr.rows();

    qe.push(arr[currIndex]);
    listofNodes.push(headNode);

    while (!qe.empty()) {
        ae_int_t* val = qe.front();
        qe.pop();
        //std::cout << "VAL:" << val[0] << " " << val[1] << std::endl;
        Node* curr = listofNodes.front();
        listofNodes.pop();

        Node* rightHead = new Node(val[1], curr, nullptr, nullptr);
        Node* leftHead = new Node(val[0], curr, nullptr, nullptr);
        curr->rightChild = rightHead;
        curr->leftChild = leftHead;

        //Right Child has a subcluster
        if (val[1] > numofRows) {
            qe.push(arr[--currIndex]);
            listofNodes.push(rightHead);
            //std::cout << "RIGHT: " << val[1] << std::endl;
        }
        //Left Child has a subcluster
        if (val[0] > numofRows) {
            qe.push(arr[--currIndex]);
            listofNodes.push(leftHead);
            //std::cout << "LEFT: " << val[0] << std::endl;
        }
        //std::cout << "INDEX: " << currIndex << " " << std::endl;
    }
}





void Clustering::getAllDistances(){
    for (int i = 0; i < num_of_agents; ++i) {
        for (int j = 0; j < num_of_agents; ++j) {
            getDistance(i, j);
        }
    }

}
int Clustering::getDistance(int a1, int a2) 
{
    if (distances[a1][a2] >= 0)
        return distances[a1][a2];
    //Mirroring
    else if (distances[a2][a1] >= 0)
        return distances[a2][a1];
    //Same value
    else if (a1 == a2) {
        distances[a1][a2] = 0;
        return 0;
    }

    assert(mdds[a1]->levels.size() == mdds[a2]->levels.size());
    int min_distance = INT_MAX;
    for (int t = 0; t < (int)mdds[a1]->levels.size(); t++)
    {
        for (const auto& node1: mdds[a1]->levels[t])
        {
            for (const auto& node2: mdds[a2]->levels[t])
            {
                int distance;
                if (lookahead == 0)
                    distance = G.get_Manhattan_distance(node1.location, node2.location);
                else if (node1.location == node2.location)
                    distance = node1.cost + node2.cost;
                else
                    distance = 3 * lookahead;
                min_distance = min(min_distance, distance);
                if (min_distance == 0)
                {
                    distances[a1][a2] = 0;
                    distances[a2][a1] = 0;
                    return 0;
                }
            }
        }
    }
    distances[a1][a2] = min_distance;
    distances[a2][a1] = min_distance;
    return min_distance;
}


void Clustering::updateLocations(const vector<State>& starts,
                                 const vector< vector<pair<int, int> > >& goal_locations)
{
    start_time = clock();
    num_of_agents = (int)starts.size();
    landmarks.resize(num_of_agents);
    mdds.resize(num_of_agents);
    distances.resize(num_of_agents, vector<int>(num_of_agents, -1));
    for (int i = 0; i < num_of_agents; i++)
    {
        landmarks[i].resize(goal_locations[i].size() + 1);
        landmarks[i][0] = starts[i].location;
        for (int j = 0; j < (int)goal_locations[i].size(); j++)
        {
            landmarks[i][j + 1] = goal_locations[i][j].first;
        }
        mdds[i] = mdd_helper.getMDD(landmarks[i]);
    }
}

//Traversal the tree in order
void Clustering:: inorderTraversal(Node* node)
{
    if (node == nullptr)
    {
        return;
    }

    inorderTraversal(node->leftChild);
    std::cout << node->val << " ";
    inorderTraversal(node->rightChild);
}

//Delete tree with root
void Clustering::deleteTree(Node* node)
{
    if (node == nullptr)
    {
        return;
    }

    deleteTree(node->leftChild);
    deleteTree(node->rightChild);
    delete node;
}