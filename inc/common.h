#pragma once
#include <utility>
#include <tuple>
#include <list>
#include <vector>
#include <iostream>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <set>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_set;
using boost::unordered_map;

using std::set;
using std::vector;
using std::tuple;
using std::deque;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::list;
using std::cout;
using std::endl;
using std::ostream;
using std::string;
using std::max;
using std::min;

//#include <boost/graph/adjacency_list.hpp>
//typedef boost::adjacency_list_traits<int, int, boost::undirectedS > confilctGraph_t;
//typedef confilctGraph_t::vertex_descriptor vertex_t;
//typedef confilctGraph_t::edge_descriptor edge_t;

enum heuristics_type { NONE, CG, DG, WDG, STRATEGY_COUNT };

typedef tuple<int, int, int, int, bool> Constraint;
typedef tuple<int, int, int, int, int> Conflict;
// typedef vector<unordered_set<std::pair<int,int> > > ConstraintTable;
typedef tuple<int, int, bool> Interval; // [t_min, t_max), have conflicts or not
#define INTERVAL_MAX 10000

ostream& operator<<(ostream& os, const Constraint& constraint);

ostream& operator<<(ostream& os, const Conflict& conflict);

ostream& operator<<(ostream& os, const Interval& interval);
