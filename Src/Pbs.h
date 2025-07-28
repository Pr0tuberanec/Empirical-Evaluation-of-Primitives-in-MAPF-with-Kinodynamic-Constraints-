#ifndef PBS_H
#define PBS_H

#include <chrono>
#include <iostream>
#include <cmath>
#include <memory>
#include <optional>
#include <stack>
#include <stdlib.h>
#include <unordered_map>
#include <vector>

#include "Agent.h"
#include "AgentInterval.h"
#include "Config.h"
#include "Constants.h"
#include "JsonLogger.h"
#include "Map.h"
#include "PTNode.h"
#include "Searchresult.h" 
#include "Sipp-ip.h"
#include "Tasks.h"

class PBS {
public:

    PBS(JsonLogger* logger, const Map& map, const Config& config, const Tasks& agent_tasks);
    ~PBS();

    SearchResult startSearch();

    const std::vector<std::vector<std::vector<Primitive>>>& getMotionPrimitives() const;

private:

    PriorityTreeNode initializeRootNode();
    bool updatePlan(PriorityTreeNode& pt_node, int idx_agent);
    SearchResult logSolution(const PriorityTreeNode& pt_node,
                             std::chrono::time_point<std::chrono::system_clock> start_time) const;
    
    std::optional<std::pair<int, int>> collisionSearch(
        const std::vector<std::shared_ptr<std::vector<Node>>>& plan) const;
    std::pair<int, int> buildInterval(const Node& node, size_t step, size_t path_size) const;
    bool hasConflict(const std::set<AgentInterval>& intervals, const AgentInterval& new_interval,
                     std::pair<int, int>& conflict_pair) const;
    
    std::vector<PriorityTreeNode> generateChildNodes(
        const PriorityTreeNode& pt_node, const std::pair<int, int>& collision);

    bool topologicalSort(int start, std::stack<int>& result,
                         const std::unordered_map<int, std::vector<int>>& less_priority,
                         std::unordered_map<int, char>& color) const;
    int cost(const PriorityTreeNode& pt_node) const;

    void buildPrimitives();
    void addTurningPrimitives(int direction);
    void addForwardPrimitives(int direction);


    Map map;
    Config config;
    Tasks agent_tasks;
    JsonLogger* logger;
    std::vector<std::vector<std::vector<Primitive>>> motion_primitives;
};

#endif