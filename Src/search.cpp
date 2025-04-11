#include "search.h"

#include <chrono>
#include <cmath>

Search::Search() {}

Search::~Search() {}

SearchResult Search::startSearch()
{
    return sresult;
}

double computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions& options)
{
    switch (options.metrictype) {
        case CN_SP_MT_EUCL: return (sqrt((i2 - i1) * (i2 - i1) + (j2 - j1) * (j2 - j1)));
        case CN_SP_MT_DIAG: return (abs(abs(i2 - i1) - abs(j2 - j1)) + sqrt(2) * (std::min(abs(i2 - i1), abs(j2 - j1))));
        case CN_SP_MT_MANH: return (abs(i2 - i1) + abs(j2 - j1));
        case CN_SP_MT_CHEB: return std::max(abs(i2 - i1), abs(j2 - j1));
        default: return 0;
    }
}

double computeCostToNeighbour(int i1, int j1, int i2, int j2)
{
    return (abs(i2 - i1) + abs(j2 - j1));
}

void Search::updateSearchResult(Node* curNode, bool found,
                                std::chrono::time_point<std::chrono::system_clock> start) {
    sresult.pathfound = found;
    if (found) {
        sresult.pathlength = curNode->g;
        makePrimaryPath(curNode);
        makeSecondaryPath();
    }

    sresult.numberofsteps = curNode->g;
    sresult.lppath = &lppath;
    sresult.hppath = &hppath;

    sresult.time = std::chrono::duration<double>(std::chrono::system_clock::now() - start).count();
}

void Search::makePrimaryPath(Node* curNode)
{
    for (Node* node = curNode; node != nullptr; node = node->getParent()) {
        lppath.emplace_back(*node);
    }

    std::reverse(lppath.begin(), lppath.end());
}

void Search::makeSecondaryPath()
{
    if (lppath.empty()) { 
        return;
    }

    int prev_dx = 0, prev_dy = 0;
    auto prev = lppath.begin();

    for (auto it = std::next(prev); it != lppath.end(); ++it) {
        int dx = it->j - prev->j;
        int dy = it->i - prev->i;

        if (dx != prev_dx || dy != prev_dy) {
            hppath.push_back(*prev);
            prev_dx = dx;
            prev_dy = dy;
        }
        prev = it;
    }
    hppath.push_back(*prev);
}
