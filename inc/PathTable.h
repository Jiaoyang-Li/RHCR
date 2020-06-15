#pragma once
#include "States.h"


class PathTable
{
public:
    PathTable(const vector<Path*>& paths, int window, int k_robust);

    void remove(const Path* old_path, int agent);
    list<std::shared_ptr<Conflict> > add(const Path* new_path, int agent);


private:

    unordered_map<int, list<pair<int, int> > > PT; // key: location; value: list of time-agent pair
    int window;
    int k_robust;
    int num_of_agents;


};

