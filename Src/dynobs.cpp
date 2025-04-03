#include "dynobs.h"

#include "astar.h"
#include "environmentoptions.h"
#include "ilogger.h"
#include "map.h"

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

std::vector<Node> convertListToVector(const std::list<std::shared_ptr<Node>>* lppath) {
    std::vector<Node> result;
    result.reserve(lppath->size());

    for (const auto& node_ptr: *lppath) {
        result.push_back(*node_ptr);
    }

    return result;
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
        obstacles_paths.emplace_back(convertListToVector(search.startSearch().lppath));
    }
}

void DynObs::AddIntervalToMaxTime(const std::unordered_map<int, int>& timesteps, int max_time) {
    for(const auto& [pos, time] : timesteps) {
        if (time < max_time) {
            free_timesteps_table[pos].emplace_back(time + 1, max_time);
        } else if (time == max_time && free_timesteps_table.find(pos) == free_timesteps_table.end()) {
            free_timesteps_table[pos].emplace_back(-1, -1); // Always closed
        }
    }
}

void DynObs::timeIntervalBuilding(const Map& map) {
    if (!obstacles_paths.empty()) {
        // auto max_time = std::max_element(obstacles_paths.begin(), obstacles_paths.end(),
        // [](const std::vector<Node>& a, const std::vector<Node>& b) {
        //     return a.size() < b.size();
        // })->size();
        //size_t max_time = map.width * map.height - 1;
        size_t max_time = 200;

        int current_cell;
        int previous_time_step;
        std::unordered_map<int, int> timesteps;
        for(size_t time = 0; time < max_time; ++time) {
            for(size_t obs_idx = 0; obs_idx < obstacles_paths.size(); ++obs_idx) {
                previous_time_step = -1;
                Node obstacle_pos = (obstacles_paths[obs_idx].size() <= time) ?
                    obstacles_paths[obs_idx].back() : obstacles_paths[obs_idx][time];
                current_cell = obstacle_pos.i * map.width + obstacle_pos.j;

                if (timesteps.find(current_cell) != timesteps.end()) {
                    previous_time_step = timesteps[current_cell];
                }

                timesteps[current_cell] = time;

                if (previous_time_step + 1 <= static_cast<int>(time) - 1) {
                    free_timesteps_table[current_cell].emplace_back(previous_time_step + 1, time - 1);
                }
            }
        }

        AddIntervalToMaxTime(timesteps, max_time - 1);
    }
}