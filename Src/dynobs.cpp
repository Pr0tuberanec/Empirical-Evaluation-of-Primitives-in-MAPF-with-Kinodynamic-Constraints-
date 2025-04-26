#include "dynobs.h"

#include "astar.h"
#include "environmentoptions.h"
#include "ilogger.h"
#include "map.h"

// Support only dynamic obstacles with one timestep moves !!!
std::pair<int, int> DynObs::generateStartOrEndDynObs(const Map& map) {
    std::uniform_int_distribution<int> distrib_y(0, map.height - 1);
    std::uniform_int_distribution<int> distrib_x(0, map.width - 1);

    int i_start = distrib_y(generator);
    int j_start = distrib_x(generator);
    while ((!map.CellIsTraversable(i_start, j_start)) ||
           ((i_start == map.start_i) && (j_start == map.start_j))) {
        i_start = distrib_y(generator);
        j_start = distrib_x(generator);
    }

    return {i_start, j_start};
}

void DynObs::createDynObstacles(ILogger* logger, Map* map, EnvironmentOptions* options,
                                int seed, int num_obs) {
    generator.seed(seed);

    for (int obs_idx = 0; obs_idx < num_obs; ++obs_idx) {
        auto [i_start, j_start] = generateStartOrEndDynObs(*map);
        auto [i_end, j_end] = generateStartOrEndDynObs(*map);
        
        Map current_obs_task(*map);
        current_obs_task.start_i = i_start;
        current_obs_task.start_j = j_start;
        current_obs_task.goal_i = i_end;
        current_obs_task.goal_j = j_end;

        Astar search;
        search.SetLogger(logger);
        search.SetMap(&current_obs_task);
        search.SetOptions(options);
        obstacles_paths.emplace_back(*(search.startSearch().lppath));
    }
}

int64_t findMaxObsTime(std::unordered_map<int, int>& stop_time, const Map& map,
                      std::unordered_set<int>& active_obstacles,
                      const std::vector<std::vector<Node>>& obstacles_paths) {
    int64_t max_obs_time = 0;
    int cnt = 0;
    for(size_t obs_idx = 0; obs_idx < obstacles_paths.size(); ++obs_idx) {
        active_obstacles.insert(obs_idx);
        max_obs_time = std::max(max_obs_time, std::ssize(obstacles_paths[obs_idx]));

        const Node& last_pos = obstacles_paths[obs_idx].back();
        int nodeIdx = last_pos.i * map.width + last_pos.j;

        // Вообще говоря у нас препятствия не должны сталкиваться
        if (stop_time.find(nodeIdx) != stop_time.end()) {
            stop_time[nodeIdx] =
                std::min(static_cast<int>(obstacles_paths[obs_idx].size()) - 1, stop_time[nodeIdx]);
        } else {
            stop_time[nodeIdx] = obstacles_paths[obs_idx].size() - 1;
        }
        ////////////////////////////////////////////////////////
    }

    return max_obs_time;
}

void DynObs::AddIntervalToMaxTime(const std::unordered_map<int, int>& timesteps,
                                  const std::unordered_map<int, int>& stop_time, int max_time) {
    for(const auto& [pos, time] : timesteps) {
        if (time < max_time && !stop_time.count(pos)) {
            free_timesteps_table[pos].emplace_back(time + 1, max_time);
        } else if (!free_timesteps_table.count(pos) &&
                   (time == max_time || time < max_time && stop_time.count(pos))) {
            free_timesteps_table[pos].emplace_back(-1, -1); // Always closed
        }
    }
}

void DynObs::timeIntervalBuilding(double max_time, const Map& map) {
    if (!obstacles_paths.empty()) {
        std::unordered_map<int, int> stop_time;
        std::unordered_set<int> active_obstacles;
        int max_obs_time = findMaxObsTime(stop_time, map, active_obstacles, obstacles_paths);

        int current_cell;
        int previous_time_step;
        std::unordered_map<int, int> timesteps;
        for(int time = 0; time < max_obs_time; ++time) {
            std::vector<int> to_remove;
            for(int obs_idx: active_obstacles) {
                if (obstacles_paths[obs_idx].size() == time + 1) {
                    to_remove.push_back(obs_idx);
                }

                previous_time_step = -1;
                Node obstacle_pos = obstacles_paths[obs_idx][time];
                current_cell = obstacle_pos.i * map.width + obstacle_pos.j;

                // Препятствия не должны сталкиваться
                obstacles_table[time * map.width * map.height + current_cell] = obs_idx;
                if (stop_time.count(current_cell) && time > stop_time[current_cell]) {
                    continue;
                }
                //////////////////////////////////////

                if (timesteps.count(current_cell)) {
                    previous_time_step = timesteps[current_cell];
                }
                timesteps[current_cell] = time;

                if (previous_time_step + 1 <= static_cast<int>(time) - 1) {
                    free_timesteps_table[current_cell].emplace_back(previous_time_step + 1, time - 1);
                }
            }

            for(int idx: to_remove) {
                active_obstacles.erase(idx);
            }
        }

        AddIntervalToMaxTime(timesteps, stop_time, max_time - 1);
    }
}