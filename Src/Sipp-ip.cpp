#include "Sipp-ip.h"

SIPPwIP::SIPPwIP(const Map& map, const Config& config, const Agent& agent,
           const std::vector<std::vector<std::vector<Primitive>>>& motion_primitives)
    : map(map), config(config), agent(agent),
      motion_primitives(motion_primitives),
      T_max(config.getSearchParam(CN_T_MAX))
{
    closed.resize(map.getHeight(),
        std::vector<std::vector<std::vector<std::set<std::pair<int, int>>>>>(
            map.getWidth(), std::vector<std::vector<std::set<std::pair<int, int>>>>(
                                CN_MXO, std::vector<std::set<std::pair<int, int>>>(
                                            CN_MXV))));
}

SIPPwIP::~SIPPwIP() {}

void SIPPwIP::clear(int height, int width) {
    open.clear();
    closed_nodes.clear();

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            for (int orientation = 0; orientation < CN_MXO; ++orientation) {
                for (int velocity = 0; velocity < CN_MXV; ++velocity) {
                    closed[row][col][orientation][velocity].clear();
                }
            }
        }
    }
}

std::pair<int, bool> SIPPwIP::search() {
    clear(map.getHeight(), map.getWidth());

    Node start_node = initializeStartNode();
    if (!validateStartNode(start_node)) {
        return {0, false};
    }
    open.insert(start_node);

    return searchLoop();
}


Node SIPPwIP::initializeStartNode() const {
    int start_i = agent.getStartI();
    int start_j = agent.getStartJ();

    int start_dir = 0;
    int start_vel = 0;
    int start_tlower = 0;

    auto it = agent.getRsrvTbl()[start_i][start_j].lower_bound({start_tlower, T_max});
    int st_tupper = it->first - 1;

    return Node(start_i, start_j, start_dir, start_vel, start_tlower, st_tupper, Primitive(),
                config.computeHFromCellToCell(start_i, start_j, agent.getGoalI(), agent.getGoalJ()));
}

bool SIPPwIP::validateStartNode(const Node& start_node) {
    int t_lower = start_node.t_lower;
    int t_upper = start_node.t_upper;

    auto it = agent.getRsrvTbl()[start_node.i][start_node.j].lower_bound({t_lower, T_max});
    --it;
    bool valid = ((t_upper >= t_lower) && (it->second < t_lower) &&
                   map.cellIsTraversable(start_node.i, start_node.j));
    if (!valid) {
        std::cerr << "Error, the initial state falls in an obstacle!!" << std::endl;
    }
    return valid;
}

std::pair<int, bool> SIPPwIP::searchLoop() {
    while (!open.empty()) {
        Node current = *open.begin();
        open.erase(open.begin());

        if (!shouldExpandNode(current)) {
            continue;
        }

        insertIntoClosed(current);
        int id = closed_nodes.size() - 1;

        if (isGoalReached(current)) {
            return {id, true};
        }

        std::vector<Node> successors;
        generateSuccessors(current, successors);
        insertSuccessors(successors, id);
    }
    return {0, false};
}

bool SIPPwIP::shouldExpandNode(Node& node) {
    int feasible_time = getEarliestFeasibleTime(node);
    if (feasible_time > node.t_upper) {
        return false;
    }

    if (node.t_lower != feasible_time) {
        node.t_lower = feasible_time;
        open.insert(node);
        return false;
    }
    return true;
}

int SIPPwIP::getEarliestFeasibleTime(const Node& node) {
    if(closed[node.i][node.j][node.dir][node.vel].empty())
        return node.t_lower;
    auto it = --closed[node.i][node.j][node.dir][node.vel].end();
    return std::max(node.t_lower, it->second + 1);
}

void SIPPwIP::insertIntoClosed(const Node& node) {
    closed[node.i][node.j][node.dir][node.vel].insert({node.t_lower, node.t_upper});
    closed_nodes.emplace_back(node);
}

bool SIPPwIP::isGoalReached(const Node& node) {
    if (!(node.i == agent.getGoalI() && node.j == agent.getGoalJ())) {
        return false;
    }

    const auto& reservations = agent.getRsrvTbl()[node.i][node.j];
    auto it = reservations.lower_bound({node.t_lower, node.t_lower});

    return it->first == (T_max + 1) && it->second == (T_max + 1);
}

void SIPPwIP::generateSuccessors(const Node& node, std::vector<Node>& successors) {
    for (const auto& primitive: motion_primitives[node.dir][node.vel]) {
        auto destinations = applyPrimitive(node, primitive);
        std::move(destinations.begin(), destinations.end(), back_inserter(successors));
    }
}

void SIPPwIP::insertSuccessors(std::vector<Node>& successors, int parent_id) {
    for (auto& successor : successors) {
        successor.parent = parent_id;
        open.insert(std::move(successor));
    }
}

std::vector<Node> SIPPwIP::applyPrimitive(const Node& node, const Primitive& primitive) {
    std::vector<std::pair<int, int>> time_intervals = {{node.t_lower, node.t_upper}};
    return propagatePrimitive(node, primitive, time_intervals);
}

std::vector<Node> SIPPwIP::propagatePrimitive(
    const Node& start_node,
    const Primitive& primitive,
    std::vector<std::pair<int, int>>& time_intervals)
{
    int i = start_node.i;
    int j = start_node.j;
    int prev_cell_touch_time = 0;
    bool end_cell_touched = false;

    for (const auto& move : primitive.moves) {
        i = start_node.i + move.dy;
        j = start_node.j + move.dx;

        if (!isReachableCell(i, j)) {
            return {};
        }

        time_intervals = intervalProjection(i, j, move, time_intervals,
            prev_cell_touch_time, end_cell_touched, primitive);
        if (time_intervals.empty()) {
            return {};
        }

        prev_cell_touch_time = move.ft_t;
        if (move.is_end_cell) {
            end_cell_touched = true;
        }
    }

    return generateSuccessorsFromIntervals(i, j, time_intervals, primitive);
}

bool SIPPwIP::isReachableCell(int i, int j) const {
    return map.cellOnGrid(i, j) && map.cellIsTraversable(i, j);
}

std::vector<std::pair<int, int>> SIPPwIP::intervalProjection(
    int i, int j,
    const Move& move,
    const std::vector<std::pair<int, int>>& inputIntervals,
    int prev_cell_touch_time,
    bool end_cell_touched,
    const Primitive& primitive)
{
    std::vector<std::pair<int, int>> projected;

    for (const auto& interval : inputIntervals) {
        int t_lower = std::min(T_max, interval.first + (move.ft_t - prev_cell_touch_time));
        int t_upper = std::min(T_max, interval.second + (move.ft_t - prev_cell_touch_time));

        auto current_rsrv_interval = agent.getRsrvTbl()[i][j].lower_bound({t_lower, T_max - 1});
        auto prev_rsrv_interval = std::prev(current_rsrv_interval);

        while (prev_rsrv_interval->second < t_upper) {
            int new_t_lower = std::max(t_lower, prev_rsrv_interval->second + 1);
            int new_t_upper = std::min(t_upper, current_rsrv_interval->first - 1 - move.sw_t);

            // Extend the reachability interval if this is the goal cell
            // and the agent's final velocity is zero (stopping condition).
            // This extension applies only to the first time the cell is reached.
            // Note: Multiple "touch times" for a single cell may occur when combining
            // movement and turning primitives to reduce search tree branching.
            if (move.is_end_cell && !end_cell_touched && primitive.end_vel == 0) {
                new_t_upper = current_rsrv_interval->first - 1 - move.sw_t;
            }

            if (new_t_lower <= new_t_upper) {
                projected.emplace_back(new_t_lower, new_t_upper);
            }

            ++current_rsrv_interval;
            ++prev_rsrv_interval;
        }
    }

    return projected;
}

std::vector<Node> SIPPwIP::generateSuccessorsFromIntervals(
    int i, int j,
    const std::vector<std::pair<int, int>>& intervals,
    const Primitive& primitive)
{
    std::vector<Node> successors;
    const auto& last_move = primitive.moves.back();

    for (const auto& interval : intervals) {
        int t_lower = std::min(T_max, interval.first + last_move.sw_t);
        int t_upper = std::min(T_max, interval.second + last_move.sw_t);

        successors.emplace_back(i, j,
                                primitive.end_dir, primitive.end_vel,
                                t_lower, t_upper, primitive,
                                config.computeHFromCellToCell(i, j, agent.getGoalI(), agent.getGoalJ()));
    }

    return successors;
}

std::vector<Node> SIPPwIP::reconstructPath(int goal_node_id) {
    std::vector<Node> path;
    while(goal_node_id != -1){
        path.push_back(closed_nodes[goal_node_id]);
        goal_node_id = closed_nodes[goal_node_id].parent;
    }
    reverse(path.begin(), path.end());

    return path;
}

std::vector<Node> SIPPwIP::expandPathWithMoves(const std::vector<Node>& path) {
    std::vector<Node> flatten_path;

    for (size_t step = 1; step < path.size(); ++step) {
        const Node& prev_node = path[step - 1];
        const Node& curr_node = path[step];
        const Primitive& primitive = curr_node.primitive;

        int last_ft = primitive.moves.back().ft_t;
        int last_sw = primitive.moves.back().sw_t;

        int primitive_start_time = std::max(
            prev_node.t_lower,
            curr_node.t_lower - last_ft - last_sw
        );

        for (const auto& move : primitive.moves) {
            int i = prev_node.i + move.dy;
            int j = prev_node.j + move.dx;

            int t_start = primitive_start_time + move.ft_t;
            int t_end = t_start + move.sw_t;

            flatten_path.emplace_back(i, j, primitive.end_vel, t_start, t_end);
        }
    }

    return flatten_path;
}
