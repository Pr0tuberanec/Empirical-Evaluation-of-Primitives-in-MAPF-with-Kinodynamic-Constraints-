#include "JsonLogger.h"

bool JsonLogger::getLog(const char* filename) {
    std::string str;
    str.append(filename);
    size_t found = str.find_last_of(".");
    if (found != std::string::npos)
        str.insert(found, "_log");
    else
        str.append("_log");
    log_filename.append(str);

    parsed_json[CNS_TAG_ROOT] = nlohmann::ordered_json::object_t{};
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG] = nlohmann::ordered_json::object_t{};
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_SUM] = nlohmann::ordered_json::object_t{};
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_MAP] = nlohmann::ordered_json::object_t{};
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_HEIGHT] = 0;
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_WIDTH] = 0;
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_AGENTS_PATHS] = nlohmann::ordered_json::object_t{};

    return true;
}

void JsonLogger::saveLog() {
    std::ofstream out(log_filename.c_str());
    if (out.is_open()) {
        out << parsed_json.dump(4);
        out.close();
        std::cout << "JSON успешно сохранен в " << log_filename << std::endl;
    } else {
        std::cout << log_filename << "<--------------" << std::endl;
        std::cerr << "Не удалось открыть файл для записи!" << std::endl;
    }
}

void JsonLogger::writeToLogSummary(float length, double time) {
    nlohmann::ordered_json summary;
    summary = {{CNS_TAG_ATTR_LENGTH, length}, {CNS_TAG_ATTR_TIME, std::to_string(time).c_str()}};
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_SUM] = summary;
}

void JsonLogger::writeToLogMap(const Map& map) {
    nlohmann::ordered_json map_json;

    for (int i = 0; i < map.getHeight(); ++i) {
        std::string str = "[";
        for (int j = 0; j < map.getWidth(); ++j) {
            str += std::to_string(map.getValue(i,j));
            str += CNS_OTHER_MATRIXSEPARATOR;
        }
        str.pop_back();
        str.push_back(']');
        map_json[i] = str;
    }
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_WIDTH] = map.getHeight();
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_HEIGHT] = map.getWidth();
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_MAP] = map_json;

}

void JsonLogger::writeToLogAgentPaths(const std::vector<std::shared_ptr<std::vector<Node>>>& agent_paths) {
    nlohmann::ordered_json agent_paths_json;
    for (size_t i = 0; i < agent_paths.size(); ++i) {
        nlohmann::ordered_json cur;
        std::string str = "agent id: " + std::to_string(i);

        for (size_t j = 0; j < agent_paths[i]->size(); ++j) {
            cur[j] = std::string(CNS_TAG_ATTR_X) + " = " + std::to_string((*(agent_paths[i]))[j].j) + ", " +
                     std::string(CNS_TAG_ATTR_Y) + " = " + std::to_string((*(agent_paths[i]))[j].i) + ", " +
                     std::string(CNS_TAG_ATTR_T_Lower) + " = " + std::to_string((*(agent_paths[i]))[j].t_lower) + ", " +
                     std::string(CNS_TAG_ATTR_T_Upper) + " = " + std::to_string((*(agent_paths[i]))[j].t_upper) + ", " +
                     std::string(CNS_TAG_VEL) + " = " + std::to_string((*(agent_paths[i]))[j].vel);
        }
        agent_paths_json[str] = cur;
    }
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_AGENTS_PATHS] = agent_paths_json;
}

void JsonLogger::writeToLogNotFound() {
    parsed_json[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_PATH] = "Path NOT found!";
}