#ifndef SIPPwIP_H
#define SIPPwIP_H

#include <chrono>
#include <cmath>
#include <set>
#include <stdlib.h>
#include <vector>
#include <unordered_map>

#include "Agent.h"
#include "Constants.h"
#include "Config.h"
#include "Map.h"
#include "Node.h"
#include "Primitive.h"

using TimeInterval = std::pair<int, int>;
using IntervalSet = std::set<TimeInterval>;
using ClosedTable = std::vector<std::vector<std::vector<std::vector<IntervalSet>>>>;

class SIPPwIP {
public:
    SIPPwIP(const Map& map, const Config& config, const Agent& agent,
         const std::vector<std::vector<std::vector<Primitive>>>& motion_primitives);
    ~SIPPwIP();

    std::pair<int, bool> search();
    std::vector<Node> reconstructPath(int goal_node_id);
    std::vector<Node> expandPathWithMoves(const std::vector<Node>& path);

private:

    void clear(int height, int width);
    Node initializeStartNode() const;
    bool validateStartNode(const Node& startNode);
    std::pair<int, bool> searchLoop();

    bool shouldExpandNode(Node& node);
    int getEarliestFeasibleTime(const Node& node);
    void insertIntoClosed(const Node& node);
    bool isGoalReached(const Node& node);

    void generateSuccessors(const Node& node, std::vector<Node>& succs);
    void insertSuccessors(std::vector<Node>& successors, int parent_id);

    std::vector<Node> applyPrimitive(const Node& node, const Primitive& primitive);
    std::vector<Node> propagatePrimitive(const Node& startNode,
        const Primitive& primitive, std::vector<std::pair<int, int>>& time_intervals);
    bool isReachableCell(int i, int j) const;
    std::vector<std::pair<int, int>> intervalProjection(
        int i, int j,
        const Move& move,
        const std::vector<std::pair<int, int>>& input_intervals,
        int prev_cell_touch_time,
        bool end_cell_touched,
        const Primitive& primitive);
    std::vector<Node> generateSuccessorsFromIntervals(int i, int j,
        const std::vector<std::pair<int, int>>& time_intervals, const Primitive& primitive);

    Map map;
    int T_max;
    Config config;
    ClosedTable closed;
    std::set<Node> open;
    std::vector<Node> closed_nodes;

    const Agent& agent;
    const std::vector<std::vector<std::vector<Primitive>>>& motion_primitives;
};

#endif