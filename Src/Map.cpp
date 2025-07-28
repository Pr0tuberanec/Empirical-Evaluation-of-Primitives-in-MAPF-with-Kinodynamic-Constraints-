#include "Map.h"

using json = nlohmann::json;

Map::Map() : height(-1), width(-1) {}

Map::~Map() {}

bool Map::getMap(const char* filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error! Failed to open file: " << filename << "\n";
        return false;
    }

    json parsed_json;
    try {
        parsed_json = json::parse(file);
    } catch (json::parse_error& e) {
        std::cerr << "Error! File is not a valid JSON: " << filename << ". Details: " << e.what() << "\n";
        return false;
    }

    if (!parsed_json.contains(CNS_TAG_ROOT) || !parsed_json[CNS_TAG_ROOT].is_object()) {
        std::cerr << "Error! Missing or invalid root tag: " << CNS_TAG_ROOT << "\n";
        return false;
    }

    const auto& map_json = parsed_json[CNS_TAG_ROOT][CNS_TAG_MAP];
    if (!map_json.contains(CNS_TAG_HEIGHT) || !map_json.contains(CNS_TAG_WIDTH) || !map_json.contains(CNS_TAG_GRID)) {
        std::cerr << "Error! Map object must contain 'height', 'width', and 'grid' tags.\n";
        return false;
    }

    try {
        height = map_json[CNS_TAG_HEIGHT].get<int>();
        width = map_json[CNS_TAG_WIDTH].get<int>();
    } catch(json::exception& e) {
        std::cerr << "Error! 'height' and 'width' must be integers. Details: " << e.what() << "\n";
        return false;
    }

    if (height <= 0 || width <= 0) {
        std::cerr << "Error! 'height' and 'width' must be positive integers.\n";
        return false;
    }

    grid = std::make_shared<std::vector<std::vector<int>>>(height, std::vector<int>(width, CN_GC_NOOBS));

    const auto& grid_json = map_json[CNS_TAG_GRID];
    if (!grid_json.is_array() || grid_json.size() != height) {
        std::cerr << "Error! 'grid' must be an array of strings with a size equal to 'height'.\n";
        return false;
    }

    for (int i = 0; i < height; ++i) {
        if (!grid_json[i].is_string()) {
            std::cerr << "Error! Each row in 'grid' must be a JSON string.\n";
            return false;
        }

        try {
            json row = json::parse(grid_json[i].get<std::string>());

            if (!row.is_array() || row.size() != width) {
                std::cerr << "Error! Row " << i << " must be a JSON array of size " << width << ".\n";
                return false;
            }

            for (int j = 0; j < width; ++j) {
                if (!row[j].is_number_integer()) {
                    std::cerr << "Error! Element at (" << i << "," << j << ") is not an integer.\n";
                    return false;
                }
                (*grid)[i][j] = row[j].get<int>();
            }
        } catch (json::parse_error& e) {
            std::cerr << "Error! Failed to parse row " << i << ": " << e.what() << "\n";
            return false;
        }
    }

    return true;
}

bool Map::cellOnGrid(int i, int j) const {
    return i >= 0 && i < height && j >= 0 && j < width;
}

bool Map::cellIsObstacle(int i, int j) const {
    return (*grid)[i][j] != CN_GC_NOOBS;
}

bool Map::cellIsTraversable(int i, int j) const {
    return (*grid)[i][j] == CN_GC_NOOBS;
}

int Map::getValue(int i, int j) const {
    if (!cellOnGrid(i, j))
        return -1;
    return (*grid)[i][j];
}

int Map::getHeight() const {
    return height;
}

int Map::getWidth() const {
    return width;
}

const std::shared_ptr<std::vector<std::vector<int>>> Map::getGrid() const {
    return grid;
}