#ifndef PTNODE_H
#define PTNODE_H

#include "Node.h"

#include <memory>
#include <vector>
#include <unordered_map>

struct PriorityTreeNode {
    std::vector<int> time;
    std::vector<std::shared_ptr<std::vector<Node>>> plan;
    std::unordered_map<int, std::vector<int>> less_priority;
    std::unordered_map<int, std::vector<int>> higher_priority;
    double cost = 0;
};

#endif