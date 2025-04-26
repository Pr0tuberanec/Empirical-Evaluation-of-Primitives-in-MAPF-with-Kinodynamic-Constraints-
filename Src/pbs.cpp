#include <iostream>
#include "node.h"
#include "pbs.h"
#include <stack>
#include <optional>
#include <unordered_map>
#include <vector>

PBS::PBS() {}
PBS::~PBS() {}

bool TopologicalSort(int start, std::stack<int>& result,
                     const std::unordered_map<int, std::vector<int>>& less_priority,
                     std::unordered_map<int, char>& color) {
    color[start] = 'g'; // grey: start process
    if (less_priority.find(start) != less_priority.end()) {
        for(auto neighbour: less_priority.at(start)) {
            if (color.find(neighbour) != color.end()) {
                if (color[neighbour] == 'b') {
                    continue;
                }
                return false;
            }
            
            if (!TopologicalSort(neighbour, result, less_priority, color)) {
                return false;
            }
        }
    }

    color[start] = 'b'; // black: end process
    result.push(start); // reminder: reverse result and clear color and result
    return true;
}

std::vector<std::vector<Node>> slice_by_indices(std::vector<std::vector<Node>>& data,
                                                std::vector<int>& indices) {
    std::vector<std::vector<Node>> result;
    result.reserve(indices.size());

    for (int idx : indices) {
        result.push_back(data[idx]);
    }

    return result;
}

std::vector<Node> FlattenVector(const std::vector<Node>& plan) {
    std::vector<Node> result;
    int j = 0;
    for (int i = 1; i < plan.size(); ++i) {
        while (j < plan[i].g) {
            result.push_back(plan[i-1]);
            result.back().g = j;
            ++j;
        }
    }
    result.push_back(plan.back());
    return result;
}

bool PBS::UpdatePlan(PriorityTreeNode& node, int idx_agent) {
    std::stack<int> topological_order;
    std::unordered_map<int, char> color;
    if (!TopologicalSort(idx_agent, topological_order, node.less_priority, color)) {
        return false;
    }

    while(!topological_order.empty()) {
        int current_agent = topological_order.top();
        topological_order.pop();

        SIPP search;
        Map current_obs_task(*map);
        current_obs_task.start_i = (*(map->agents_tasks))[current_agent].start_i;
        current_obs_task.start_j = (*(map->agents_tasks))[current_agent].start_j;
        current_obs_task.goal_i = (*(map->agents_tasks))[current_agent].goal_i;
        current_obs_task.goal_j = (*(map->agents_tasks))[current_agent].goal_j;

        search.SetLogger(logger);
        search.SetMap(&current_obs_task);
        search.SetOptions(options);

        DynObs dyn_obs_creator;
        std::vector<std::vector<Node>> obstacles = slice_by_indices(node.plan, node.higher_priority[current_agent]);
        dyn_obs_creator.SetObstaclesPaths(obstacles);
        dyn_obs_creator.timeIntervalBuilding(T_max, *map);
        search.SetTMax(T_max);
        search.SetPtrFreeTimestepsTable(dyn_obs_creator.GetPtrFreeTimestepsTable());
        search.SetPtrObstaclesTable(dyn_obs_creator.GetPtrObsTable());
        search.SetPtrObstaclesPaths(dyn_obs_creator.GetPtrObstaclesPaths());

        auto paths = slice_by_indices(node.plan, node.higher_priority[current_agent]);
        paths.push_back(node.plan[current_agent]);
        std::vector<int> collision = HasNoCollision(paths);
        if (collision[0] == -1) {
            continue;
        }


        SearchResult result = search.startSearch();
        //std::cout << (result.lppath)->size() << std::endl;
        //std::cout << (result.lppath)->size() << std::endl;
        // std::cout << current_agent << std::endl;
        // for (auto [first, second]: *dyn_obs_creator.GetPtrFreeTimestepsTable()) {
        //     std::cout << "Point " << first / 5 << " " << first % 5 << std::endl;
        //     for (auto elem: second) {
        //         std::cout << elem.first << " " << elem.second << std::endl;
        //     }
        // }
        // for (auto i: *(result.lppath)) {
        //     std::cout << i.j << " " << i.i << std::endl;
        // }

        if (!result.pathfound) {
            return false;
        }
        *(result.lppath) = FlattenVector(*(result.lppath));
        node.plan[current_agent] = *(result.lppath);
        node.time[current_agent] = result.pathlength;
    }

    return true;
}

int cost(PriorityTreeNode& node) {
    int sum = 0;
    for(size_t idx = 0; idx < node.time.size(); ++idx) {
        sum += node.time[idx];
    }
    return sum;
}

std::vector<int> PBS::HasNoCollision(const std::vector<std::vector<Node>>& plan) {
    std::unordered_map<int, int> points;
    for(int idx_agent = 0; idx_agent < plan.size(); ++idx_agent) {
        for(int idx_pos = 0; idx_pos < plan[idx_agent].size(); ++idx_pos) {
            int nodeIdx = idx_pos * map->width * map->height + plan[idx_agent][idx_pos].i *
                          map->width + plan[idx_agent][idx_pos].j;
            if (points.count(nodeIdx)) {
                return {points[nodeIdx], idx_agent};
            }
            points[nodeIdx] = idx_agent;
        }
    }

    for(int idx_agent = 0; idx_agent < plan.size(); ++idx_agent) {
        for(int idx_pos = 1; idx_pos < plan[idx_agent].size(); ++idx_pos) {
            int oldNodeIdx = (idx_pos - 1) * map->width * map->height + plan[idx_agent][idx_pos].i *
                             map->width + plan[idx_agent][idx_pos].j;
            int newNodeIdx = idx_pos * map->width * map->height + plan[idx_agent][idx_pos - 1].i *
                             map->width + plan[idx_agent][idx_pos - 1].j;
            if (points.count(oldNodeIdx) && points.count(newNodeIdx) && points[oldNodeIdx] == points[newNodeIdx] && points[oldNodeIdx] != idx_agent) {
                return {points[newNodeIdx], idx_agent};
            } 
        }
    }

    for(int idx_end_point = 0; idx_end_point < plan.size(); ++idx_end_point) {
        int time = plan[idx_end_point].back().g;
        int end_pos_i = plan[idx_end_point].back().i;
        int end_pos_j = plan[idx_end_point].back().j;
        
        for(int idx_agent = 0; idx_agent < plan.size(); ++idx_agent) {
            if (idx_agent == idx_end_point) {
                continue;
            }

            for(int idx_pos = time; idx_pos < plan[idx_agent].size(); ++idx_pos) {
                if (plan[idx_agent][idx_pos].i == end_pos_i &&
                    plan[idx_agent][idx_pos].j == end_pos_j) {
                        return {idx_end_point, idx_agent};
                }
            }
        }
    }

    return {-1, -1};
}

SearchResult PBS::startSearch() {
    PriorityTreeNode root;
    auto start = std::chrono::system_clock::now();
    for(int idx_agent = 0; idx_agent < map->agents_tasks->size(); ++idx_agent) {

        Map current_obs_task(*map);
        current_obs_task.start_i = (*(map->agents_tasks))[idx_agent].start_i;
        current_obs_task.start_j = (*(map->agents_tasks))[idx_agent].start_j;
        current_obs_task.goal_i = (*(map->agents_tasks))[idx_agent].goal_i;
        current_obs_task.goal_j = (*(map->agents_tasks))[idx_agent].goal_j;

        Astar search;
        search.SetLogger(logger);
        search.SetMap(&current_obs_task);
        search.SetOptions(options);
        SearchResult result = search.startSearch();
        if (!result.pathfound) {
            return SearchResult();
        }
        root.plan.emplace_back(*(result.lppath));
        root.time.push_back(result.pathlength);
    }

    root.cost = cost(root);
    PriorityTreeNode current_node;
    std::stack<PriorityTreeNode> open;
    open.push(root);
    while (!open.empty()) {
        current_node = open.top();
        open.pop();

        std::vector<int> collision = HasNoCollision(current_node.plan);
        if (collision[0] == -1) {
            Node dummy;
            dummy.g = current_node.cost;
            updateSearchResult(&dummy, true, start);
            logger->writeToLogObstaclesPath(&current_node.plan);
            return sresult;
        }

        std::vector<PriorityTreeNode> nodes;
        for(int second_priority_agent: collision) {
            PriorityTreeNode new_node;

            new_node.plan = current_node.plan;

            new_node.less_priority = current_node.less_priority;
            new_node.higher_priority = current_node.higher_priority;
            int first_priority_agent = collision[0] == second_priority_agent ?
                                       collision[1] : collision[0];
            new_node.less_priority[first_priority_agent].push_back(second_priority_agent);
            new_node.higher_priority[second_priority_agent].push_back(first_priority_agent);
            new_node.time = current_node.time;

            bool success = UpdatePlan(new_node, second_priority_agent);
            if (success) {
                new_node.cost = cost(new_node);
                nodes.push_back(new_node);
            }
        }

        if (nodes.size() == 2) {
            if (nodes[0].cost < nodes[1].cost) {
                open.push(nodes[1]);
                open.push(nodes[0]);
            } else {
                open.push(nodes[0]);
                open.push(nodes[1]);
            }
        } else if (nodes.size() == 1) {
            open.push(nodes[0]);
        }
    }

    return SearchResult();
}