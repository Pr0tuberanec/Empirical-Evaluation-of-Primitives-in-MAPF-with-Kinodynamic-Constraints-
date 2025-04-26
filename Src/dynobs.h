#ifndef DynObs_H
#define DynObs_H
#include <utility>
#include <random>

#include "environmentoptions.h"
#include "ilogger.h"
#include "node.h"

class DynObs {
public:
    DynObs() {}
    ~DynObs() {}

    void createDynObstacles(ILogger* logger, Map* map, EnvironmentOptions* options,
                            int seed, int num_obs);
    std::pair<int, int> generateStartOrEndDynObs(const Map& map);

    void AddIntervalToMaxTime(const std::unordered_map<int, int>& timesteps,
                              const std::unordered_map<int, int>& stop_time, int max_time);
    void timeIntervalBuilding(double max_time, const Map& map);

    std::vector<std::vector<Node>>* GetPtrObstaclesPaths() { return &obstacles_paths; }
    std::unordered_map<int, std::vector<std::pair<int, int>>>* GetPtrFreeTimestepsTable() {
        return &free_timesteps_table;
    }
    std::unordered_map<int, int>* GetPtrObsTable() { return &obstacles_table; }
    void SetObstaclesPaths(std::vector<std::vector<Node>>& paths) {
        obstacles_paths = paths;
    }

private:
    std::mt19937 generator;
    std::unordered_map<int, int> obstacles_table;
    std::vector<std::vector<Node>> obstacles_paths;
    std::unordered_map<int, std::vector<std::pair<int, int>>> free_timesteps_table;
};

#endif