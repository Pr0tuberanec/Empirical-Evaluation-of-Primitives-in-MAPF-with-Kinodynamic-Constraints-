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

    TimeNode startNode(map->start_i, map->start_j, 0,
                       computeHFromCellToCell(map->start_i, map->start_j, map->goal_i, map->goal_j, *options),
                       hweight, start_t, end_t, 0);

    open.Add(startNode);
}

bool SIPP::isNodeInOpen(int nodeIdx, int start_t) {
    return open.GetKey().count(nodeIdx) && open.GetKey().at(nodeIdx).count(start_t);
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

        if (isNodeInOpen(childIdx, child.start_t)) {
            if (open.GetComparator()(open.GetData().at(open.GetKey().at(childIdx).at(child.start_t)), child)) {
                open.UpdateKey(open.GetData().at(open.GetKey().at(childIdx).at(child.start_t)), child);
            }
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

// Плохо продумана поддержка передвижений различной стоимости
bool SIPP::checkMoveIntersection(TimeNode curNode, Node newNode, int start_t, int end_t) {
    for(int time = start_t; time < end_t; ++time) {
        int curNodeIdx = time * map->width * map->height + newNode.i * map->width + newNode.j;
        int newNodeIdx = (time + 1) * map->width * map->height + curNode.i * map->width + curNode.j;
        if (obstacles_table->count(curNodeIdx) && obstacles_table->count(newNodeIdx)) {
            if ((*obstacles_table)[curNodeIdx] == (*obstacles_table)[newNodeIdx]) {
                return true;
            }
        }
    }
    return false;
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

void SIPP::getSuccessors(TimeNode curNode, std::vector<TimeNode>& successors) {
    successors.clear();
    std::vector<std::pair<int, int>> side_moves = {{0, -1}, {0, 1}, {1, 0}, {-1, 0}}; //, Point(0, 0)};
    for (const auto& move : side_moves) {
        Node neighbour(curNode.i + move.first, curNode.j + move.second, -1, -1, -1);
        int neighbourIdx = neighbour.i * map->width + neighbour.j;

        if (!map->CellOnGrid(neighbour.i, neighbour.j) ||
            !map->CellIsTraversable(neighbour.i, neighbour.j)) {
            continue;
        }
        
        auto free_timesteps = getFreeTimesteps(neighbourIdx);
        for (const auto& [start_t, end_t] : free_timesteps) {
            if (!isValidSuccessor(neighbourIdx, start_t)) {
                continue;
            }

            int dist = computeCostToNeighbour(curNode.i, curNode.j, neighbour.i, neighbour.j);
            if (start_t > curNode.end_t + dist || end_t < curNode.t + dist) {
                continue;
            }
            
            int l_bnd_t = std::max(curNode.t + dist, start_t);
            int r_bnd_t = std::min(end_t, curNode.end_t + dist);

            if (checkMoveIntersection(curNode, neighbour, curNode.end_t, l_bnd_t)) {
                continue;
            }

            TimeNode newNode(neighbour.i, neighbour.j, curNode.g + dist,
                             computeHFromCellToCell(neighbour.i, neighbour.j, map->goal_i, map->goal_j, *options), 
                             hweight, start_t, end_t, l_bnd_t, nullptr);
            successors.emplace_back(newNode);
        }
    }
}