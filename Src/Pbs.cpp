#include "Pbs.h"

PBS::PBS(JsonLogger* logger, const Map& map, const Config& config, const Tasks& agent_tasks)
    : logger(logger), map(map), config(config), agent_tasks(agent_tasks) {
    motion_primitives.resize(CN_MXO,
                             std::vector<std::vector<Primitive>>(CN_MXV));
}

PBS::~PBS() {}

SearchResult PBS::startSearch() {
    buildPrimitives();
    auto start_time = std::chrono::high_resolution_clock::now();

    PriorityTreeNode root = initializeRootNode();
    if (root.plan.empty()) {
        return SearchResult();
    }

    std::stack<PriorityTreeNode> open;
    open.push(root);

    while (!open.empty()) {
        PriorityTreeNode current_pt_node = open.top();
        open.pop();

        auto collision = collisionSearch(current_pt_node.plan);
        if (!collision.has_value()) {
            return logSolution(current_pt_node, start_time);
        }

        std::vector<PriorityTreeNode> children = generateChildNodes(current_pt_node, *collision);
        for (const auto& child : children) {
            open.push(child);
        }
    }

    return SearchResult();
}

PriorityTreeNode PBS::initializeRootNode() {
    PriorityTreeNode root;

    for (int agent_id = 0; agent_id < agent_tasks.getAgentsNoConstRef().size(); ++agent_id) {
        auto& agent = agent_tasks.getAgentsNoConstRef()[agent_id];
        agent.initializeReservationTable(map, config.getSearchParam(CN_T_MAX));
        SIPPwIP planner(map, config, agent, motion_primitives);

        auto result = planner.search();
        if (!result.second) {
            return PriorityTreeNode();
        }

        std::vector<Node> path = planner.reconstructPath(result.first);
        std::vector<Node> flatten_path = planner.expandPathWithMoves(path);

        root.time.push_back(flatten_path.back().t_upper);
        root.plan.push_back(std::make_shared<std::vector<Node>>(std::move(flatten_path)));
    }

    root.cost = cost(root);
    return root;
}

std::optional<std::pair<int, int>> PBS::collisionSearch(
    const std::vector<std::shared_ptr<std::vector<Node>>>& plan) const {
    std::unordered_map<int, std::set<AgentInterval>> visited_intervals;

    for (int agent_id = 0; agent_id < plan.size(); ++agent_id) {
        const auto& path = plan[agent_id];

        for (size_t step = 0; step < path->size(); ++step) {
            const Node& node = (*path)[step];
            int cell_id = node.i * map.getWidth() + node.j;
            auto step_interval = buildInterval(node, step, path->size());
            AgentInterval add_interval{step_interval, agent_id};

            std::pair<int, int> conflict_pair;
            auto& intervals = visited_intervals[cell_id];
            if (hasConflict(intervals, add_interval, conflict_pair))
                return {conflict_pair};

            intervals.insert(add_interval);
        }
    }

    return std::nullopt;;
}

std::pair<int, int> PBS::buildInterval(const Node& node, size_t step, size_t path_size) const {
    int t_start = node.t_lower;
    int t_end = (step == path_size - 1) ? config.getSearchParam(CN_T_MAX) : node.t_upper;
    return {t_start, t_end};
}

bool PBS::hasConflict(const std::set<AgentInterval>& intervals,
                      const AgentInterval& add_interval,
                      std::pair<int, int>& conflict_pair) const {
    auto it = intervals.lower_bound(add_interval);

    if (it != intervals.begin()) {
        auto prev = std::prev(it);
        if (prev->interval.second >= add_interval.interval.first &&
            prev->agent_id != add_interval.agent_id) {
            conflict_pair = {prev->agent_id, add_interval.agent_id};
            return true;
        }
    }

    if (it != intervals.end()) {
        if (it->interval.first <= add_interval.interval.second &&
            it->agent_id != add_interval.agent_id) {
            conflict_pair = {it->agent_id, add_interval.agent_id};
            return true;
        }
    }

    return false;
}

std::vector<PriorityTreeNode> PBS::generateChildNodes(
    const PriorityTreeNode& pt_node, const std::pair<int, int>& collision) {
    std::vector<PriorityTreeNode> children;

    for (int lower_agent : {collision.first, collision.second}) {
        int higher_agent = (lower_agent == collision.first) ? collision.second : collision.first;
        PriorityTreeNode child = pt_node;

        child.less_priority[higher_agent].push_back(lower_agent);
        child.higher_priority[lower_agent].push_back(higher_agent);

        if (updatePlan(child, lower_agent)) {
            child.cost = cost(child);
            children.push_back(child);
        }
    }

    if (children.size() == 2 && children[0].cost < children[1].cost) {
        std::swap(children[0], children[1]);
    }

    return children;
}

bool PBS::updatePlan(PriorityTreeNode& pt_node, int idx_agent) {
    std::stack<int> order;
    std::unordered_map<int, char> color;
    if (!topologicalSort(idx_agent, order, pt_node.less_priority, color)) {
        return false;
    }

    while (!order.empty()) {
        int current = order.top();
        order.pop();

        SIPPwIP planner(map, config, agent_tasks.getAgentsNoConstRef()[current], motion_primitives);

        Agent& agent = agent_tasks.getAgentsNoConstRef()[current];

        agent.clrReservationTable(map, config.getSearchParam(CN_T_MAX));
        agent.buildReservationTable(pt_node.plan, pt_node.higher_priority[current]);
        agent.normalizeReservationTable();

        auto result = planner.search();
        if (!result.second) {
            return false;
        }

        std::vector<Node> path = planner.reconstructPath(result.first);
        std::vector<Node> flatten_path = planner.expandPathWithMoves(path);

        pt_node.time[current] = flatten_path.back().t_upper;
        pt_node.plan[current] = std::make_shared<std::vector<Node>>(std::move(flatten_path));
    }

    return true;
}

bool PBS::topologicalSort(int agent_id, std::stack<int>& order,
                          const std::unordered_map<int, std::vector<int>>& less_priority,
                          std::unordered_map<int, char>& color) const {
    color[agent_id] = 'g'; // grey: start process
    if (less_priority.find(agent_id) != less_priority.end()) {
        for(auto neighbour: less_priority.at(agent_id)) {
            if (color.find(neighbour) != color.end()) {
                if (color[neighbour] == 'b') {
                    continue;
                }
                return false;
            }
            
            if (!topologicalSort(neighbour, order, less_priority, color)) {
                return false;
            }
        }
    }

    color[agent_id] = 'b'; // black: end process
    order.push(agent_id);
    return true;
}

int PBS::cost(const PriorityTreeNode& pt_node) const {
    int sum = 0;
    for (int t : pt_node.time) sum += t;
    return sum;
}

SearchResult PBS::logSolution(const PriorityTreeNode& pt_node,
    std::chrono::time_point<std::chrono::system_clock> start_time) const {
    SearchResult search_results;
    search_results.pathfound = true;
    search_results.time = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::high_resolution_clock::now() - start_time).count();
    search_results.pathlength = pt_node.cost;
    logger->writeToLogAgentPaths(pt_node.plan);
    return search_results;
}

void PBS::buildPrimitives() {
    for (int direction = 0; direction < CN_MXO; ++direction) {
        addTurningPrimitives(direction);
        addForwardPrimitives(direction);
    }
}

 // =================================================================================
 //                                 First Family Of Primitives
 // =================================================================================

void PBS::addTurningPrimitives(int direction) {
    Primitive turn_right;
    turn_right.moves = {Move(0, 0, 0, 0, 1)};
    turn_right.end_dir = (direction + 1) % CN_MXO;
    turn_right.end_vel = 0;
    motion_primitives[direction][0].emplace_back(turn_right);

    Primitive turn_left = turn_right;
    turn_left.end_dir = (direction + 3) % CN_MXO;
    motion_primitives[direction][0].emplace_back(turn_left);
}

void PBS::addForwardPrimitives(int direction) {
    int dx[4] = {0, -1, 0, 1};
    int dy[4] = {1, 0, -1, 0};

    std::vector<std::pair<int, int>> time_cost_pairs = {std::make_pair(0,28), std::make_pair(0,28)};
    int total_cost = 28;
    int sz = time_cost_pairs.size();

    Primitive celerate;
    celerate.end_dir = direction;
    celerate.end_vel = 0;
    for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
        celerate.moves.emplace_back(
            dy[direction] * segment,
            dx[direction] * segment,
            time_cost_pairs[segment].first,
            time_cost_pairs[segment].second,
            segment == time_cost_pairs.size() - 1
        );
    }
    motion_primitives[direction][0].emplace_back(celerate);

}
    
 // =================================================================================
 //                                 Second Family Of Primitives
 // =================================================================================

    // void PBS::addTurningPrimitives(int direction) {
    //     Primitive turn_right;
    //     turn_right.moves = {Move(0, 0, 0, 0, 1)};
    //     turn_right.end_dir = (direction + 1) % CN_MXO;
    //     turn_right.end_vel = 0;
    //     motion_primitives[direction][0].emplace_back(turn_right);

    //     Primitive turn_left = turn_right;
    //     turn_left.end_dir = (direction + 3) % CN_MXO;
    //     motion_primitives[direction][0].emplace_back(turn_left);

    //     Primitive turn_right_1;
    //     turn_right_1.moves = {Move(0, 0, 0, 0, 1)};
    //     turn_right_1.end_dir = (direction + 1) % CN_MXO;
    //     turn_right_1.end_vel = 1;
    //     motion_primitives[direction][1].emplace_back(turn_right_1);

    //     Primitive turn_left_1 = turn_right_1;
    //     turn_left_1.end_dir = (direction + 3) % CN_MXO;
    //     motion_primitives[direction][1].emplace_back(turn_left_1);

    //     Primitive turn_right_2;
    //     turn_right_2.moves = {Move(0, 0, 0, 0, 1)};
    //     turn_right_2.end_dir = (direction + 1) % CN_MXO;
    //     turn_right_2.end_vel = 2;
    //     motion_primitives[direction][2].emplace_back(turn_right_2);

    //     Primitive turn_left_2 = turn_right_2;
    //     turn_left_2.end_dir = (direction + 3) % CN_MXO;
    //     motion_primitives[direction][2].emplace_back(turn_left_2);
    // }

    // void PBS::addForwardPrimitives(int direction) {
    //     int dx[4] = {0, -1, 0, 1};
    //     int dy[4] = {1, 0, -1, 0};

    //     std::vector<std::pair<int, int>> time_cost_pairs = {std::make_pair(0,20), std::make_pair(0,29), std::make_pair(20, 15), std::make_pair(28,12), std::make_pair(34,6)};
    //     int total_cost = 40;
    //     int sz = time_cost_pairs.size();

    //     // Acceleration primitive (velocity 0 -> 2)
    //     Primitive accelerate;
    //     accelerate.end_dir = direction;
    //     accelerate.end_vel = 2;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         accelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             time_cost_pairs[segment].first,
    //             time_cost_pairs[segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][0].emplace_back(accelerate);

    //     // Deceleration primitive (velocity 2 -> 0)
    //     Primitive decelerate;
    //     decelerate.end_dir = direction;
    //     decelerate.end_vel = 0;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         int rev_segment = time_cost_pairs.size() - 1 - segment;
    //         decelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             total_cost - (time_cost_pairs[rev_segment].first + time_cost_pairs[rev_segment].second),
    //             time_cost_pairs[rev_segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][2].emplace_back(decelerate);

    //     // Constant velocity primitive (velocity 2 -> 2)
    //     Primitive keep_velocity;
    //     keep_velocity.end_dir = direction;
    //     keep_velocity.end_vel = 2;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 5, 0),
    //         Move(dy[direction], dx[direction], 0, 5, 1)
    //     };
    //     motion_primitives[direction][2].emplace_back(keep_velocity);

    //     // Accelerate primitive (velocity 0 -> 1)
    //     time_cost_pairs = {std::make_pair(0,20), std::make_pair(0,20)};
    //     total_cost = 20;
    //     sz = time_cost_pairs.size();

    //     accelerate.moves.clear();
    //     accelerate.end_dir = direction;
    //     accelerate.end_vel = 1;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         accelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             time_cost_pairs[segment].first,
    //             time_cost_pairs[segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][0].emplace_back(accelerate);

    //     // Deceleration primitive (velocity 1 -> 0)
    //     decelerate.moves.clear();
    //     decelerate.end_dir = direction;
    //     decelerate.end_vel = 0;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         int rev_segment = time_cost_pairs.size() - 1 - segment;
    //         decelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             total_cost - (time_cost_pairs[rev_segment].first + time_cost_pairs[rev_segment].second),
    //             time_cost_pairs[rev_segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][1].emplace_back(decelerate);

    //     // Constant velocity primitive (velocity 1 -> 1)
    //     keep_velocity.moves.clear();
    //     keep_velocity.end_dir = direction;
    //     keep_velocity.end_vel = 1;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 10, 0),
    //         Move(dy[direction], dx[direction], 0, 10, 1)
    //     };
    //     motion_primitives[direction][1].emplace_back(keep_velocity);

    // }

 // =================================================================================
 //                                 Third Family Of Primitives
 // =================================================================================

    // void PBS::addTurningPrimitives(int direction) {
    //     Primitive turn_right;
    //     turn_right.moves = {Move(0, 0, 0, 0, 1)};
    //     turn_right.end_dir = (direction + 1) % CN_MXO;
    //     turn_right.end_vel = 0;
    //     motion_primitives[direction][0].emplace_back(turn_right);

    //     Primitive turn_left = turn_right;
    //     turn_left.end_dir = (direction + 3) % CN_MXO;
    //     motion_primitives[direction][0].emplace_back(turn_left);

    //     Primitive turn_right_1;
    //     turn_right_1.moves = {Move(0, 0, 0, 0, 1)};
    //     turn_right_1.end_dir = (direction + 1) % CN_MXO;
    //     turn_right_1.end_vel = 1;
    //     motion_primitives[direction][1].emplace_back(turn_right_1);

    //     Primitive turn_left_1 = turn_right_1;
    //     turn_left_1.end_dir = (direction + 3) % CN_MXO;
    //     motion_primitives[direction][1].emplace_back(turn_left_1);

    //     Primitive turn_right_2;
    //     turn_right_2.moves = {Move(0, 0, 0, 0, 1)};
    //     turn_right_2.end_dir = (direction + 1) % CN_MXO;
    //     turn_right_2.end_vel = 2;
    //     motion_primitives[direction][2].emplace_back(turn_right_2);

    //     Primitive turn_left_2 = turn_right_2;
    //     turn_left_2.end_dir = (direction + 3) % CN_MXO;
    //     motion_primitives[direction][2].emplace_back(turn_left_2);
    // }

    // void PBS::addForwardPrimitives(int direction) {
    //     int dx[4] = {0, -1, 0, 1};
    //     int dy[4] = {1, 0, -1, 0};

    //     std::vector<std::pair<int, int>> time_cost_pairs = {std::make_pair(0,20), std::make_pair(0,29), std::make_pair(20, 15), std::make_pair(28,12), std::make_pair(34,6)};
    //     int total_cost = 40;
    //     int sz = time_cost_pairs.size();

    //     // Acceleration primitive (velocity 0 -> 2)
    //     Primitive accelerate;
    //     accelerate.end_dir = direction;
    //     accelerate.end_vel = 2;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         accelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             time_cost_pairs[segment].first,
    //             time_cost_pairs[segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][0].emplace_back(accelerate);

    //     // Deceleration primitive (velocity 2 -> 1)
    //     time_cost_pairs = {std::make_pair(20, 9), std::make_pair(20, 15), std::make_pair(28,12), std::make_pair(34,6)};
    //     total_cost = 40;
    //     sz = time_cost_pairs.size();

    //     Primitive decelerate;
    //     decelerate.end_dir = direction;
    //     decelerate.end_vel = 1;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         int rev_segment = time_cost_pairs.size() - 1 - segment;
    //         decelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             total_cost - (time_cost_pairs[rev_segment].first + time_cost_pairs[rev_segment].second),
    //             time_cost_pairs[rev_segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][2].emplace_back(decelerate);

    //     // Constant velocity primitive (velocity 2 -> 2)
    //     Primitive keep_velocity;
    //     keep_velocity.end_dir = direction;
    //     keep_velocity.end_vel = 2;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 5, 0),
    //         Move(dy[direction], dx[direction], 0, 5, 1)
    //     };
    //     motion_primitives[direction][2].emplace_back(keep_velocity);

    //     // Accelerate primitive (velocity 0 -> 1)
    //     time_cost_pairs = {std::make_pair(0,20), std::make_pair(0,20)};
    //     total_cost = 20;
    //     sz = time_cost_pairs.size();

    //     accelerate.moves.clear();
    //     accelerate.end_dir = direction;
    //     accelerate.end_vel = 1;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         accelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             time_cost_pairs[segment].first,
    //             time_cost_pairs[segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][0].emplace_back(accelerate);

    //     // Deceleration primitive (velocity 1 -> 0)
    //     decelerate.moves.clear();
    //     decelerate.end_dir = direction;
    //     decelerate.end_vel = 0;
    //     for (int segment = 0; segment < time_cost_pairs.size(); ++segment) {
    //         int rev_segment = time_cost_pairs.size() - 1 - segment;
    //         decelerate.moves.emplace_back(
    //             dy[direction] * segment,
    //             dx[direction] * segment,
    //             total_cost - (time_cost_pairs[rev_segment].first + time_cost_pairs[rev_segment].second),
    //             time_cost_pairs[rev_segment].second,
    //             segment == time_cost_pairs.size() - 1
    //         );
    //     }
    //     motion_primitives[direction][1].emplace_back(decelerate);

    //     // Constant velocity primitive (velocity 1 -> 1)
    //     keep_velocity.moves.clear();
    //     keep_velocity.end_dir = direction;
    //     keep_velocity.end_vel = 1;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 10, 0),
    //         Move(dy[direction], dx[direction], 0, 10, 1)
    //     };
    //     motion_primitives[direction][1].emplace_back(keep_velocity);

    //     // turn with forward move
    //     keep_velocity.moves.clear();
    //     keep_velocity.end_dir = (direction + 1) % CN_MXO;
    //     keep_velocity.end_vel = 1;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 13, 0),
    //         Move(dy[direction], dx[direction], 0, 16, 0),
    //         Move(dy[(direction + 1) % CN_MXO], dx[(direction + 1) % CN_MXO], 0, 16, 0),
    //         Move(dy[direction] + dy[(direction + 1) % CN_MXO], dx[direction] + dx[(direction + 1) % CN_MXO], 3, 13, 1)
    //     };
    //     motion_primitives[direction][1].emplace_back(keep_velocity);

    //     keep_velocity.moves.clear();
    //     keep_velocity.end_dir = (direction + 3) % CN_MXO;
    //     keep_velocity.end_vel = 1;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 13, 0),
    //         Move(dy[direction], dx[direction], 0, 16, 0),
    //         Move(dy[(direction + 3) % CN_MXO], dx[(direction + 3) % CN_MXO], 0, 16, 0),
    //         Move(dy[direction] + dy[(direction + 3) % CN_MXO], dx[direction] + dx[(direction + 3) % CN_MXO], 3, 13, 1)
    //     };
    //     motion_primitives[direction][1].emplace_back(keep_velocity);

    //     // forward move with turn
    //     keep_velocity.moves.clear();
    //     keep_velocity.end_dir = direction;
    //     keep_velocity.end_vel = 1;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 12, 0),
    //         Move(dy[(direction + 1) % CN_MXO], dx[(direction + 1) % CN_MXO], 0, 12, 0),
    //         Move(dy[direction], dx[direction], 0, 21, 0),
    //         Move(dy[direction] + dy[(direction + 1) % CN_MXO], dx[direction] + dx[(direction + 1) % CN_MXO], 3, 24, 0),
    //         Move(2*dy[direction], 2*dx[direction], 12, 24, 0),
    //         Move(2*dy[direction] + dy[(direction + 1) % CN_MXO], 2*dx[direction] + dx[(direction + 1) % CN_MXO], 12, 24, 1)
    //     };
    //     motion_primitives[direction][1].emplace_back(keep_velocity);

    //     keep_velocity.moves.clear();
    //     keep_velocity.end_dir = direction;
    //     keep_velocity.end_vel = 1;
    //     keep_velocity.moves = {
    //         Move(0, 0, 0, 12, 0),
    //         Move(dy[(direction + 3) % CN_MXO], dx[(direction + 3) % CN_MXO], 0, 12, 0),
    //         Move(dy[direction], dx[direction], 0, 21, 0),
    //         Move(dy[direction] + dy[(direction + 3) % CN_MXO], dx[direction] + dx[(direction + 3) % CN_MXO], 3, 24, 0),
    //         Move(2*dy[direction], 2*dx[direction], 12, 24, 0),
    //         Move(2*dy[direction] + dy[(direction + 3) % CN_MXO], 2*dx[direction] + dx[(direction + 3) % CN_MXO], 12, 24, 1)
    //     };
    //     motion_primitives[direction][1].emplace_back(keep_velocity);

    // }

 // =================================================================================
 //                                 
 // =================================================================================

const std::vector<std::vector<std::vector<Primitive>>>& PBS::getMotionPrimitives() const {
    return motion_primitives;
}