#include "Tasks.h"

using json = nlohmann::json;

bool Tasks::getTasks(const char* filename, int map_height, int map_width) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error! Failed to open file: " << filename << "\n";
        return false;
    }

    if (!json::accept(file)) {
        std::cerr << "Error! File is not a valid JSON: " << filename << "\n";
        return false;
    }

    file.clear();
    file.seekg(0);

    nlohmann::ordered_json parsed_json = nlohmann::ordered_json::parse(std::ifstream(filename));

    nlohmann::ordered_json root_json;
    if (!parsed_json.contains(CNS_TAG_ROOT)) {
        std::cerr << "Error! No '" << CNS_TAG_ROOT << "' element found in JSON file!" << std::endl;
        return false;
    }
    root_json = parsed_json[CNS_TAG_ROOT];

    if (!root_json.contains(CNS_TAG_MAP)) {
        std::cerr << "Error! No '" << CNS_TAG_MAP << "' tag found in JSON file!" << std::endl;
        return false;
    }
    nlohmann::ordered_json map_json = root_json[CNS_TAG_MAP];

    nlohmann::ordered_json agents_tasks_json = map_json[CNS_TAG_AGENTS];
    for (auto it = agents_tasks_json.begin(); it != agents_tasks_json.end(); ++it) {
        const std::string& agent_id = it.key();
        const auto& agent_task = it.value();

        if (!agent_task.contains(CNS_TAG_STX) || !agent_task.contains(CNS_TAG_STY) ||
            !agent_task.contains(CNS_TAG_FINX) || !agent_task.contains(CNS_TAG_FINY)) {
            std::cerr << "Error! Agent '" << agent_id << "' is missing one or more required fields." << std::endl;
            return false;
        }

        int sx = agent_task[CNS_TAG_STX];
        int sy = agent_task[CNS_TAG_STY];
        int fx = agent_task[CNS_TAG_FINX];
        int fy = agent_task[CNS_TAG_FINY];

        if (sx < 0 || sx >= map_width || fx < 0 || fx >= map_width ||
            sy < 0 || sy >= map_height || fy < 0 || fy >= map_height) {
            std::cerr << "Error! Agent '" << agent_id << "' has coordinates out of bounds." << std::endl;
            return false;
        }
        agents.emplace_back(sy, sx, fy, fx);
    }

    return true;
}

std::vector<Agent>& Tasks::getAgentsNoConstRef() {
    return agents;
}