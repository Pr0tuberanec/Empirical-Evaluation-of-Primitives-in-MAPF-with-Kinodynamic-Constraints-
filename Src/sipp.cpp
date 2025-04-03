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
                (*free_timesteps_table)[start_pos][0].second : map->height * map->width - 1;

    TimeNode startNode(map->start_i, map->start_j, 0,
                       computeHFromCellToCell(map->start_i, map->start_j, map->goal_i, map->goal_j, *options),
                       hweight, start_t, end_t, 0);

    open.Add(startNode);
}

bool SIPP::isNodeInOpen(int nodeIdx, int start_t) {
    return open.GetKey().count(nodeIdx) && open.GetKey().at(nodeIdx).count(start_t);
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
        if (curNode.i == map->goal_i && curNode.j == map->goal_j) {
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

// Нужно перейти на рёберную проверку пересечений
bool SIPP::checkIntersection(TimeNode curNode, Node newNode, int cur_time) {
    if (!obstacles_paths->empty()) {
        for (size_t obs_idx = 0; obs_idx < obstacles_paths->size(); ++obs_idx) {
            const auto& obs_path = (*obstacles_paths)[obs_idx];

            if (cur_time + 1 < obs_path.size()) {
                if ((obs_path[cur_time + 1].i == curNode.i && obs_path[cur_time + 1].j == curNode.j) &&
                    (obs_path[cur_time].i == newNode.i && obs_path[cur_time].j == newNode.j)) {
                    return true;
                }
            }
        }
    }
    return false;
}


std::vector<std::pair<int, int>> SIPP::getFreeTimesteps(int nodeIdx) {
    
    if (free_timesteps_table->find(nodeIdx) == free_timesteps_table->end()) {
        return {{0, map->height * map->width - 1}};
    }
    return (*free_timesteps_table)[nodeIdx];
}

bool SIPP::isValidSuccessor(int nodeIdx, int start_t) {
    
    return close.find(nodeIdx) == close.end() || close[nodeIdx].find(start_t) == close[nodeIdx].end();
}

int SIPP::findEarliestAvailableTime(TimeNode& curNode, Node& tmp, int l_bnd_t, int r_bnd_t) {
    int time = l_bnd_t - 1;
    while ((time < r_bnd_t) && checkIntersection(curNode, tmp, time)) {
        ++time;
    }
    return (time == r_bnd_t) ? -1 : ++time;
}

void SIPP::getSuccessors(TimeNode curNode, std::vector<TimeNode>& successors) {
    successors.clear();
    std::vector<std::pair<int, int>> side_moves = {{0, -1}, {0, 1}, {1, 0}, {-1, 0}}; //, Point(0, 0)};
    for (const auto& move : side_moves) {
        Node tmp(curNode.i + move.first, curNode.j + move.second, curNode.g,
                 computeHFromCellToCell(map->start_i, map->start_j, map->goal_i, map->goal_j, *options),
                 hweight);
        int nodeIdx = tmp.i * map->width + tmp.j;

        if (!map->CellOnGrid(tmp.i, tmp.j) || !map->CellIsTraversable(tmp.i, tmp.j)) {
            continue;
        }
        
        auto free_timesteps = getFreeTimesteps(nodeIdx);
        for (const auto& [start_t, end_t] : free_timesteps) {
            if (!isValidSuccessor(nodeIdx, start_t)) {
                continue;
            }

            if (start_t > curNode.end_t + 1 || end_t < curNode.t + 1) {
                continue;
            }

            int l_bnd_t = (start_t <= curNode.t + 1) ? curNode.t + 1 : start_t;
            int r_bnd_t = (end_t <= curNode.t + 1) ? end_t : curNode.end_t + 1;

            int earliest_time = findEarliestAvailableTime(curNode, tmp, l_bnd_t, r_bnd_t);
            if (earliest_time == -1) {
                continue;
            }

            TimeNode newNode(tmp.i, tmp.j, curNode.g + 1,
                             computeHFromCellToCell(tmp.i, tmp.j, map->goal_i, map->goal_j, *options), 
                             hweight, start_t, end_t, earliest_time, nullptr);
            successors.emplace_back(newNode);
        }
    }
}