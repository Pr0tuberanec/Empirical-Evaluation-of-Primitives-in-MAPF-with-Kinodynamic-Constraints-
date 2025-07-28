#include "Config.h"

using json = nlohmann::json;

Config::Config() {}

Config::~Config() {}

bool Config::getConfig(const char *filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Error opening file!" << std::endl;
        return false;
    }

    json parsed_json;
    try {
        file >> parsed_json;
    } catch (const std::exception& e) {
        std::cout << "Error parsing JSON: " << e.what() << std::endl;
        return false;
    }

    if (!parsed_json.contains(CNS_TAG_ROOT)) {
        std::cerr << "No '" << CNS_TAG_ROOT << "' element found." << std::endl;
        return false;
    }
    auto root_json = parsed_json[CNS_TAG_ROOT];

    search_params = std::make_shared<std::vector<int>>(CN_SEARCH_PARAMS_COUNT, 0);  

    auto algorithm = root_json.value(CNS_TAG_ALG, json::object());
    std::string metric = algorithm.value(CNS_TAG_MT, CNS_SP_MT_EUCL);
    std::transform(metric.begin(), metric.end(), metric.begin(), ::tolower);

    if (metric == CNS_SP_MT_MANH) (*search_params)[CN_SP_MT] = CN_SP_MT_MANH;
    else if (metric == CNS_SP_MT_EUCL) (*search_params)[CN_SP_MT] = CN_SP_MT_EUCL;
    else if (metric == CNS_SP_MT_DIAG) (*search_params)[CN_SP_MT] = CN_SP_MT_DIAG;
    else if (metric == CNS_SP_MT_CHEB) (*search_params)[CN_SP_MT] = CN_SP_MT_CHEB;
    else {
        std::cerr << "Warning: Unknown metric type, defaulting to 'euclidean'" << std::endl;
        (*search_params)[CN_SP_MT] = CN_SP_MT_EUCL;
    }

    if (!algorithm.contains(CNS_T_MAX)) {
        std::cerr << "Warning: '" << CNS_T_MAX << "' not specified, using default value 1000" << std::endl;
    }
    (*search_params)[CN_T_MAX] = algorithm.value(CNS_T_MAX, 1000);

    return true;
}

double Config::computeHFromCellToCell(int i1, int j1, int i2, int j2) const {
    switch ((*search_params)[CN_SP_MT]) {
        case CN_SP_MT_EUCL: return (sqrt((i2 - i1) * (i2 - i1) + (j2 - j1) * (j2 - j1)));
        case CN_SP_MT_DIAG: return (abs(abs(i2 - i1) - abs(j2 - j1)) + sqrt(2) * (std::min(abs(i2 - i1), abs(j2 - j1))));
        case CN_SP_MT_MANH: return (abs(i2 - i1) + abs(j2 - j1));
        case CN_SP_MT_CHEB: return std::max(abs(i2 - i1), abs(j2 - j1));
        default: return 0;
    }
}

int Config::getSearchParam(int index) const {
    return (*search_params)[index];
}