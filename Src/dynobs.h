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

    void AddIntervalToMaxTime(const std::unordered_map<int, int>& timesteps, int max_time);
    void timeIntervalBuilding(const Map& map);

    std::vector<std::vector<Node>>* GetPtrObstaclesPaths() { return &obstacles_paths; }
    std::unordered_map<int, std::vector<std::pair<int, int>>>* GetPtrFreeTimestepsTable() {
        return &free_timesteps_table;
    }

private:
    std::mt19937 generator;
    std::vector<std::vector<Node>> obstacles_paths;
    std::unordered_map<int, std::vector<std::pair<int, int>>> free_timesteps_table;
};

#endif