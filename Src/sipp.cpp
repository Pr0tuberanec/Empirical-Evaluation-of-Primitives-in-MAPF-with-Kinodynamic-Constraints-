#include "sipp.h"
#include "search.h"
#include "heap.h"
#include <chrono>
#include <limits>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <iostream>

SIPP::SIPP(double w, int BT)
{
    hweight = w;
    breakingties = BT;
}

SIPP::SIPP() {}

SIPP::~SIPP() {}

bool SIPP::stopCriterion() {
    if (open.GetSize() == 0) {
        std::cout << "OPEN heap is empty!" << std::endl;
        return true;
    }
    return false;
}

void SIPP::initializeSearch() {
    open = BinHeap(map->width);
    close.clear();
    lppath.clear();
    hppath.clear();
    sresult = SearchResult();

    int start_pos = map->start_i * map->width + map->start_j;
    int start_t = free_timesteps_table->count(start_pos) ?
                  (*free_timesteps_table)[start_pos][0].first : 0;
    int end_t = free_timesteps_table->count(start_pos) ?
                (*free_timesteps_table)[start_pos][0].second : T_max - 1;

    TimeNode startNode(map->start_i, map->start_j, start_t,
                       computeHFromCellToCell(map->start_i, map->start_j, map->goal_i, map->goal_j, *options),
                       hweight, start_t, end_t, 0, 0);

    open.Add(startNode);
}

bool SIPP::isNodeInOpen(int nodeIdx, int start_t) {
    return open.GetKey().count(nodeIdx) && open.GetKey().at(nodeIdx).count(start_t);
}

bool SIPP::isNodeInClose(int nodeIdx, int start_t) {
    return close.count(nodeIdx) && close.at(nodeIdx).count(start_t);
}

bool SIPP::isGoalReachable(const TimeNode& curNode) {
    if (curNode.i != map->goal_i || curNode.j != map->goal_j)
        return false;
    
    int goalIdx = map->goal_i * map->width + map->goal_j;
    auto it = free_timesteps_table->find(goalIdx);
    if (it == free_timesteps_table->end()) // No obstacles in goal node by the all time
        return true;

    const auto& last = it->second.back();
    return last.first <= curNode.t && last.second == (T_max - 1);
}

void SIPP::processNode(const TimeNode& curNode) {
    std::vector<TimeNode> successors;
    close[curNode.i * map->width + curNode.j][curNode.start_t] = curNode;
    getSuccessors(curNode, successors);

    for(TimeNode& child : successors) {
        child.parent = &close[curNode.i * map->width + curNode.j][curNode.start_t];
        int childIdx = child.i * map->width + child.j;

        if (isNodeInOpen(childIdx, child.start_t) || isNodeInClose(childIdx, child.start_t)) {
            continue;
        } else {
            open.Add(child);
        }
    }
}

void SIPP::updateSearchResult(Node* curNode, bool found,
    std::chrono::time_point<std::chrono::system_clock> start) {
    sresult.nodescreated = open.GetSize() + close.size();
    Search::updateSearchResult(curNode, found, start);
}

SearchResult SIPP::startSearch() {
    initializeSearch();

    auto start = std::chrono::system_clock::now();
    bool found = false;
    TimeNode curNode;

    while (!stopCriterion()) {
        curNode = getMin();
        if (isGoalReachable(curNode)) {
            found = true;
            break;
        }
        processNode(curNode);
    }

    updateSearchResult(&curNode, found, start);
    logger->writeToLogObstaclesPath(obstacles_paths);
    return sresult;
}

TimeNode SIPP::getMin()
{
    TimeNode min = open.GetMin();
    open.ExtractMin();
    return min;
}

std::vector<std::pair<int, int>> SIPP::getFreeTimesteps(int nodeIdx) {
    if (free_timesteps_table->find(nodeIdx) == free_timesteps_table->end()) {
        return {{0, T_max - 1}};
    }
    return (*free_timesteps_table)[nodeIdx];
}

bool SIPP::isValidSuccessor(int nodeIdx, int start_t) {
    return close.find(nodeIdx) == close.end() || close[nodeIdx].find(start_t) == close[nodeIdx].end();
}

// Плохо продумана поддержка передвижений различной стоимости
bool SIPP::checkMoveIntersection(int curNode_i, int curNode_j, int newNode_i, int newNode_j, int start_t, int end_t) {
    for(int time = start_t; time < end_t; ++time) {
        int curNodeIdx = time * map->width * map->height + newNode_i * map->width + newNode_j;
        int newNodeIdx = (time + 1) * map->width * map->height + curNode_i * map->width + curNode_j;
        if (obstacles_table->count(curNodeIdx) && obstacles_table->count(newNodeIdx)) {
            if ((*obstacles_table)[curNodeIdx] == (*obstacles_table)[newNodeIdx]) {
                return true;
            }
        }
    }
    return false;
}

std::pair<int, int> SIPP::searchIntrvl(int nodeIdx, std::pair<int, int> intrvl) {
    const std::vector<std::pair<int, int>>& free_intrvls = getFreeTimesteps(nodeIdx);
    int r_bnd = free_intrvls.size();
    int l_bnd = -1;
    while (r_bnd - l_bnd > 1) {
        int mid = (r_bnd + l_bnd) / 2;
        if (free_intrvls[mid].first > intrvl.first) {
            r_bnd = mid;
        } else {
            l_bnd = mid;
        }
    }

    return free_intrvls[l_bnd];
}   

void SIPP::getSuccessors(TimeNode curNode, std::vector<TimeNode>& successors) {
    successors.clear();
    std::vector<std::pair<int, int>> side_moves = {{0, -1}, {0, 1}, {1, 0}, {-1, 0}}; //, Point(0, 0)};
    for (auto move : side_moves) {
        Primitive edge(curNode, move, map);
        TimeNode neighbour(curNode.i + move.first, curNode.j + move.second, -1, -1, -1, -1, -1, -1, edge.end_vel);
        int neighbourIdx = neighbour.i * map->width + neighbour.j;

        auto intrvls = projectIntervals(edge, curNode, map);
        if (neighbour.vel == 0) {
            for(auto& intrvl : intrvls) {
                auto upper_intrvl = searchIntrvl(neighbourIdx, intrvl);
                intrvl.second = upper_intrvl.second;
            }
        }

        for(auto intrvl : intrvls) {
            TimeNode newNode(neighbour.i, neighbour.j, intrvl.first,
                computeHFromCellToCell(neighbour.i, neighbour.j, map->goal_i, map->goal_j, *options), 
                hweight, intrvl.first, intrvl.second, intrvl.first, neighbour.vel, nullptr);
            successors.emplace_back(newNode);
        }
    }
}

std::vector<std::pair<int, int>> SIPP::projectIntervals(Primitive edge, TimeNode startNode, Map* map) {
    std::vector<std::pair<int ,int>> time_intrvls{{startNode.start_t, startNode.end_t}};
    double t = 0;
    auto prev_cell = edge.cells[0];
    for(auto cell : edge.cells) {
        if (!map->CellOnGrid(cell.i, cell.j) ||
            !map->CellIsTraversable(cell.i, cell.j)) {
            return {};
        }

        std::vector<std::pair<int, int>> new_intrvls;
        double delta = cell.intrvl.first - t;
        t = cell.intrvl.first;

        for(std::pair<int, int> ti : time_intrvls) {
            for(std::pair<int ,int> si : getFreeTimesteps(cell.nodeIdx)) {//(*free_timesteps_table)[cell.nodeIdx]) {
                double t_earliest = std::max<double>(ti.first + delta, si.first);
                double t_latest = std::min<double>(ti.second + delta, si.second - (cell.intrvl.second - cell.intrvl.first));
                if (t_earliest <= t_latest) {
                    int curNode_i = prev_cell.nodeIdx / map->width;
                    int curNode_j = prev_cell.nodeIdx % map->width;
                    int newNode_i = cell.nodeIdx / map->width;
                    int newNode_j = cell.nodeIdx % map->width;
                    if (checkMoveIntersection(curNode_i, curNode_j, newNode_i, newNode_j, ti.second, si.first)) {
                        break;
                    }
                    new_intrvls.emplace_back(t_earliest, t_latest);
                }
            }
        }
        prev_cell = cell;
        time_intrvls = std::move(new_intrvls);
        new_intrvls.clear();
    }

    Cell last_cell = edge.cells.back();
    double delta = edge.cost - last_cell.intrvl.first;
    for(auto& ti : time_intrvls) {
        ti.first += delta;
        ti.second += delta;
    }

    return time_intrvls;
}