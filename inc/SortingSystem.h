#pragma once
#include "BasicSystem.h"
#include "SortingGraph.h"


class SortingSystem :
	public BasicSystem
{
public:
    int c; // param for induct assignment

	SortingSystem(const SortingGrid& G, MAPFSolver& solver);
    ~SortingSystem();

    void simulate(int simulation_time);

private:
	const SortingGrid& G;

    // record usage of induct stations
    boost::unordered_map<int, int> drives_in_induct_stations; // induct location + #drives that intends to go to this induct station

	void initialize();

    // assign tasks
    void initialize_start_locations();
    void initialize_goal_locations();
	void update_goal_locations();

    int assign_induct_station(int curr) const;
    int assign_eject_station() const;
};

