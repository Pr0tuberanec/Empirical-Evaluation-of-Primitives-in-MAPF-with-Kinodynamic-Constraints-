#include "astar.h"
#include "environmentoptions.h"
#include "map.h"
#include "search.h"

#include <chrono>

Astar::Astar() {}

Astar::Astar(double w, int BT)
{
    hweight = w;
    breakingties = BT;
}

Astar::~Astar() {}

void Astar::getSuccessors(const Node& curNode) {
    static const std::vector<std::pair<int, int>> side_moves = {{0, -1}, {0, 1}, {1, 0}, {-1, 0}};

    for (const auto& move : side_moves) {
        Node neighbour(curNode.i + move.first, curNode.j + move.second, curNode.g +
                       computeCostToNeighbour(curNode.i, curNode.j, curNode.i + move.first,
                                              curNode.j + move.second),
                       computeHFromCellToCell(curNode.i + move.first, curNode.j + move.second,
                                              map->goal_i, map->goal_j, *options));

        neighbour.F = neighbour.g + neighbour.H;
        neighbour.parent = &(close.find(curNode.i * map->width + curNode.j)->second);

        if (map->CellOnGrid(neighbour.i, neighbour.j) &&
            map->CellIsTraversable(neighbour.i, neighbour.j)) {
            open.push(neighbour);
        }
    }
}

SearchResult Astar::startSearch() {
    auto start = std::chrono::system_clock::now();
    Node curNode(map->start_i, map->start_j, 0,
                 computeHFromCellToCell(map->start_i, map->start_j, map->goal_i, map->goal_j, *options));
    open.push(curNode);

    bool found = false;
    while (!open.empty()) {
        curNode = open.top();
        open.pop();

        int nodeIdx = curNode.i * map->width + curNode.j;
        if (close.find(nodeIdx) != close.end()) {
            continue;
        }

        close[nodeIdx] = curNode;
        if (curNode.i == map->goal_i && curNode.j == map->goal_j) {
            found = true;
            break;
        }
        getSuccessors(curNode);
    }

    updateSearchResult(&curNode, found, start);
    return sresult;
}

void Astar::updateSearchResult(Node* curNode, bool found,
                               std::chrono::time_point<std::chrono::system_clock> start) {
    sresult.nodescreated = open.size() + close.size();
    Search::updateSearchResult(curNode, found, start);
}