#pragma once
#include "BasicGraph.h"

class Clustering {
public:
    void run(); //Wooju

private:
    int num_of_agents;
    vector<int> start_locations;

    int getDistance(int a1, int a2); // Jiaoyang will do this
};

