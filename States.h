#pragma once
#include "common.h"

struct State
{
    int location;
    int timestep;
    int orientation;

    State wait() const {return State(location, timestep + 1, orientation); }

    struct Hasher
    {
        std::size_t operator()(const State& n) const
        {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t time_hash = std::hash<int>()(n.timestep);
            size_t ori_hash = std::hash<int>()(n.orientation);
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2));
        }
    };

    void operator = (const State& other)
    {
        timestep = other.timestep;
        location = other.location;
        orientation = other.orientation;
    }

    bool operator == (const State& other) const
    {
        return timestep == other.timestep && location == other.location && orientation == other.orientation;
    }

    bool operator != (const State& other) const
    {
        return timestep != other.timestep || location != other.location || orientation != other.orientation;
    }

    State(): location(-1), timestep(-1), orientation(-1) {}
    // State(int loc): loc(loc), timestep(0), orientation(0) {}
    // State(int loc, int timestep): loc(loc), timestep(timestep), orientation(0) {}
    State(int location, int timestep = -1, int orientation = -1):
            location(location), timestep(timestep), orientation(orientation) {}
    State(const State& other) {location = other.location; timestep = other.timestep; orientation = other.orientation; }
};

std::ostream & operator << (std::ostream &out, const State &s);


typedef std::vector<State> Path;

std::ostream & operator << (std::ostream &out, const Path &path);