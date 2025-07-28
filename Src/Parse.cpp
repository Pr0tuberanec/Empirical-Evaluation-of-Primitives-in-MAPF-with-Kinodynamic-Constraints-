#include "Parse.h"

using ordered_json = nlohmann::ordered_json;

static ordered_json parseMap(const fs::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open map file");

    std::string line;
    int h = 0, w = 0;
    while (std::getline(in, line)) {
        if (line.starts_with("height")) h = std::stoi(line.substr(6));
        else if (line.starts_with("width")) w = std::stoi(line.substr(5));
        else if (line == "map") break;
    }

    ordered_json j{{"height", h}, {"width", w}, {"grid", ordered_json::array()}};

    while (std::getline(in, line)) {
        std::ostringstream row;
        row << "[";
        for (size_t i = 0; i < line.size(); ++i)
            row << ((line[i] == '.' || line[i] == ' ' || line[i] == 'G') ? "0" : "1")
                << (i + 1 < line.size() ? "," : "");
        row << "]";
        j["grid"].push_back(row.str());
    }
    return j;
}

static ordered_json parseScen(const fs::path& path, int num_agents = 5) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open scen file");

    std::string line;
    std::getline(in, line); // skip header

    ordered_json agents;
    int id = 0;

    while (std::getline(in, line) && id < num_agents) {
        std::istringstream iss(line);
        std::vector<std::string> tok{std::istream_iterator<std::string>{iss}, {}};
        if (tok.size() < 8) continue;

        agents["id_" + std::to_string(id)] = {
            {"startx", std::stoi(tok[4])},
            {"starty", std::stoi(tok[5])},
            {"finishx", std::stoi(tok[6])},
            {"finishy", std::stoi(tok[7])}
        };
        ++id;
    }
    return agents;
}

static ordered_json parseConfig(const fs::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open config file");
    ordered_json cfg;
    in >> cfg;
    return cfg;
}

fs::path buildJsonFromMapScen(const fs::path& map, const fs::path& scen,
                              const fs::path& config, int num_agents) {
    auto map_json = parseMap(map);
    auto agents = parseScen(scen, num_agents);
    auto cfg = parseConfig(config);

    ordered_json root = {
        {"root", {
            {"map", {
                {"width", map_json["width"]},
                {"height", map_json["height"]},
                {"grid", map_json["grid"]},
                {"agents", agents}
            }},
            {"algorithm", cfg}
        }}
    };

    fs::path out = map.parent_path() /
                   (map.stem().string() + "-k" + std::to_string(num_agents) + ".json");
    std::ofstream(out) << root.dump(4);
    return out;
}

fs::path prepareTaskFromArgs(int argc, char* argv[], int& num_agents)
{
    namespace fs = std::filesystem;
    fs::path task;

    if (argc == 2 && fs::path(argv[1]).extension() == ".json") {
        task = argv[1];
    }
    else if (argc >= 3) {
        fs::path map_path = argv[1];
        fs::path scen_path = argv[2];
        fs::path config_path = "/home/user/config.json";

        if (argc >= 4) {
            if (fs::path(argv[3]).extension() == ".json") {
                config_path = argv[3];
            } else {
                num_agents = std::stoi(argv[3]);
            }
        }

        if (argc == 5) {
            num_agents = std::stoi(argv[4]);
        }

        try {
            task = buildJsonFromMapScen(map_path, scen_path, config_path, num_agents);
        } catch (const std::exception& e) {
            std::cerr << "Error while building task JSON: " << e.what() << std::endl;
            return {};
        }
    }
    else {
        std::cerr << "Usage:\n  ./Search task.json\n  ./Search map.map scen.scen [config.json] [num_agents]" << std::endl;
        return {};
    }

    return task;
}
