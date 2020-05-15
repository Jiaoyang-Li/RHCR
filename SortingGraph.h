#pragma once
#include "BasicGraph.h"


class SortingGrid :
	public BasicGraph
{
public:
    unordered_map<std::string, int> inducts;
    unordered_map<string, list<int> > ejects; // one eject station could have multiple eject fiducials

    bool load_map(string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
};
