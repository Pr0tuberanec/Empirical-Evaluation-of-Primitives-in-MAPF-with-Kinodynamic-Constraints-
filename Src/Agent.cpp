#include "Agent.h"

Agent::Agent(int si, int sj, int gi, int gj)
    : start_i(si), start_j(sj), goal_i(gi), goal_j(gj) {}

void Agent::initializeReservationTable(const Map& map, int T_max) {
    rsrv_tbl.assign(map.getHeight(), std::vector<std::set<std::pair<int, int>>>(
                                         map.getWidth(), {{-1, -1}, {T_max + 1, T_max + 1}}));
}

void Agent::buildReservationTable(const std::vector<std::shared_ptr<std::vector<Node>>>& paths,
                                  const std::vector<int>& indices) {

    for(const auto& idx : indices) {
        const auto& obstacle_path = *paths[idx];
        for(int i = 0; i + 1 < obstacle_path.size(); ++i) {
            const Node& node = obstacle_path[i];
            rsrv_tbl[node.i][node.j].insert({node.t_lower, node.t_upper});
        }

        const Node& last_node = obstacle_path.back();
        rsrv_tbl[last_node.i][last_node.j].insert(
            {last_node.t_lower,
             std::prev(rsrv_tbl[last_node.i][last_node.j].end())->first});
    }
}

void Agent::normalizeReservationTable() {
    for (auto& row : rsrv_tbl) {
        for (auto& cell : row) {
            unionIntervals(cell);
        }
    }
}

void Agent::unionIntervals(std::set<std::pair<int, int>>& intervals) const {
    
    std::vector<std::pair<int, int>> merged_intervals;
    std::vector<std::pair<int, int>> temp_intervals(intervals.begin(), intervals.end());
    merged_intervals.push_back(temp_intervals[0]);

    for (size_t i = 1; i < temp_intervals.size(); ++i) {
        if (temp_intervals[i].first <= merged_intervals.back().second) {
            merged_intervals.back().second = std::max(merged_intervals.back().second, temp_intervals[i].second);
        } else {
            merged_intervals.push_back(temp_intervals[i]);
        }
    }

    intervals.clear();
    for (const auto& interval : merged_intervals) {
        intervals.insert(interval);
    }
}

void Agent::clrReservationTable(const Map& map, int T_max) {
    for (auto& row : rsrv_tbl) {
        for (auto& cell : row) {
            cell.clear();
            cell.insert({-1, -1});
            cell.insert({T_max + 1, T_max + 1});
        }
    }
}

int Agent::getStartI() const { return start_i; }
int Agent::getStartJ() const { return start_j; }
int Agent::getGoalI() const { return goal_i; }
int Agent::getGoalJ() const { return goal_j; }

const std::vector<std::vector<std::set<std::pair<int, int>>>>& Agent::getRsrvTbl() const {
    return rsrv_tbl;
}