#pragma once
#include "common.h"
#include "States.h"

#define WEIGHT_MAX INT_MAX/2


class BasicGraph
{
public:
    vector<std::string> types;
    unordered_map<int, vector<double>> heuristics;
    virtual ~BasicGraph()= default;
    string map_name;
	virtual bool load_map(string fname) = 0;
    list<State> get_neighbors(const State& v) const;
    list<int> get_neighbors(int v) const;
    list<State> get_reverse_neighbors(const State& v) const; // ignore time
    double get_weight(int from, int to) const; // fiducials from and to are neighbors
    vector<vector<double> > get_weights() const {return weights; }
    int get_rotate_degree(int dir1, int dir2) const; // return 0 if it is 0; return 1 if it is +-90; return 2 if it is 180

    void print_map() const;
    int get_rows() const { return rows; }
    int get_cols() const { return cols; }
    int size() const { return rows * cols; }

    bool valid_move(int loc, int dir) const {return (weights[loc][dir] < WEIGHT_MAX - 1); }
    int get_Manhattan_distance(int loc1, int loc2) const;
    int move[4];
    void copy(const BasicGraph& copy);
    int get_direction(int from, int to) const;

	vector<double> compute_heuristics(int root_location); // compute distances from all lacations to the root location
	bool load_heuristics_table(std::ifstream& myfile);
	void save_heuristics_table(string fname);

    int rows;
    int cols;
    vector<vector<double> > weights; // (directed) weighted 4-neighbor grid
    bool consider_rotation;
};
