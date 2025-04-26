#ifndef PBS_H
#define PBS_H
#include "gl_const.h"
#include "astar.h"
#include "sipp.h"
#include "dynobs.h"
#include "ilogger.h"
#include "search.h"

#include <math.h>
#include <stdlib.h>

struct PriorityTreeNode {
    std::vector<int> time;
    std::vector<std::vector<Node>> plan;
    std::unordered_map<int, std::vector<int>> less_priority;
    std::unordered_map<int, std::vector<int>> higher_priority;
    double cost;

    // bool operator<(const PriorityTreeNode& other) const { // non-increase
    //     return cost < other.cost;
    // }
};

class PBS : public Search
{
    public:
        PBS();
        ~PBS();
        SearchResult startSearch();
    
    private:
        bool UpdatePlan(PriorityTreeNode& node, int idx_agent);
        std::vector<int> HasNoCollision(const std::vector<std::vector<Node>>& plan);
};

#endif