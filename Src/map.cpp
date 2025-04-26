#include "map.h"
#include "json.hpp"
#include <fstream>
#include <set>
#include <random>
#include <iostream>

using json = nlohmann::json;
Map::Map()
{
    height = -1;
    width = -1;
    start_i = -1;
    start_j = -1;
    goal_i = -1;
    goal_j = -1;
    agents_tasks = nullptr;
    Grid = nullptr;
    cellSize = 1;
}

Map::~Map() {}

bool Map::CellIsTraversable(int i, int j) const
{
    return (Grid[i][j] == CN_GC_NOOBS);
}

bool Map::CellIsObstacle(int i, int j) const
{
    return (Grid[i][j] != CN_GC_NOOBS);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

bool Map::getMap(const char *FileName)
{
    int grid_i = 0, grid_j = 0;

    std::stringstream stream;

    bool hasGridMem = false, hasGrid = false, hasHeight = false, hasWidth = false, hasCellSize = false;

    std::string value;
    std::ifstream file(FileName);
    if (!json::accept(file))
    {
        std::cout << "Error opening JSON file!" << std::endl;
        return false;
    }
    nlohmann::ordered_json objJson = nlohmann::ordered_json::parse(std::ifstream(FileName));

    nlohmann::ordered_json root;
    if (!objJson.contains(CNS_TAG_ROOT)) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' element found in JSON file!" << std::endl;
        return false;
    }
    root = objJson[CNS_TAG_ROOT];

    // Get MAP element
    if (!root.contains(CNS_TAG_MAP)) {
        std::cout << "Error! No '" << CNS_TAG_MAP << "' tag found in JSON file!" << std::endl;
        return false;
    }
    nlohmann::ordered_json map = root[CNS_TAG_MAP];

    for (auto element = map.begin(); element != map.end(); element++) {

        stream.str("");
        stream.clear();

        if(element.key() != CNS_TAG_GRID)
        {
            if (!element.value().is_string())
                value = to_string(element.value());
            else
                value = element.value();
            stream << value;
        }
        value = element.key();


        if (!hasGridMem && hasHeight && hasWidth) { 
            Grid = std::shared_ptr<std::shared_ptr<int[]>[]>(new std::shared_ptr<int[]>[height]);
        
            for (int i = 0; i < height; ++i) {
                Grid[i] = std::shared_ptr<int[]>(new int[width]());
            }
        
            hasGridMem = true;
        }

        if (value == CNS_TAG_HEIGHT) {
            if (hasHeight) {
                std::cout << "Warning! Duplicate '" << CNS_TAG_HEIGHT << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_HEIGHT << "' =" << height << "will be used."
                          << std::endl;
            }
            else {
                if (!((stream >> height) && (height > 0))) {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_HEIGHT
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_HEIGHT << "' tag should be an integer >=0" << std::endl;
                    std::cout << "Continue reading JSON and hope correct value of '" << CNS_TAG_HEIGHT
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                    hasHeight = true;
            }
        }
        else if (value == CNS_TAG_WIDTH) {
            if (hasWidth) {
                std::cout << "Warning! Duplicate '" << CNS_TAG_WIDTH << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_WIDTH << "' =" << width << "will be used." << std::endl;
            }
            else {
                if (!((stream >> width) && (width > 0))) {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_WIDTH
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_WIDTH << "' tag should be an integer AND >0" << std::endl;
                    std::cout << "Continue reading JSON and hope correct value of '" << CNS_TAG_WIDTH
                              << "' tag will be encountered later..." << std::endl;

                }
                else
                    hasWidth = true;
            }
        }
        else if (value == CNS_TAG_CELLSIZE) {
            if (hasCellSize) {
                std::cout << "Warning! Duplicate '" << CNS_TAG_CELLSIZE << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_CELLSIZE << "' =" << cellSize << "will be used."
                          << std::endl;
            }
            else {
                if (!((stream >> cellSize) && (cellSize > 0))) {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_CELLSIZE
                              << "' tag encountered (or could not convert to double)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_CELLSIZE
                              << "' tag should be double AND >0. By default it is defined to '1'" << std::endl;
                    std::cout << "Continue reading JSON and hope correct value of '" << CNS_TAG_CELLSIZE
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                    hasCellSize = true;
            }
        } else if (value == CNS_TAG_AGENTS) {
            agents_tasks = std::make_shared<std::vector<Agent>>();
            nlohmann::ordered_json agents = map[CNS_TAG_AGENTS];
            for (auto it = agents.begin(); it != agents.end(); ++it) {
                const std::string& agent_id = it.key();
                const auto& agent_data = it.value();
        
                if (!agent_data.contains(CNS_TAG_STX) || !agent_data.contains(CNS_TAG_STY) ||
                    !agent_data.contains(CNS_TAG_FINX) || !agent_data.contains(CNS_TAG_FINY)) {
                    std::cout << "Error! Agent '" << agent_id << "' is missing one or more required fields." << std::endl;
                    return false;
                }
        
                int sx = agent_data[CNS_TAG_STX];
                int sy = agent_data[CNS_TAG_STY];
                int fx = agent_data[CNS_TAG_FINX];
                int fy = agent_data[CNS_TAG_FINY];
        
                if (sx < 0 || sx >= width || fx < 0 || fx >= width ||
                    sy < 0 || sy >= height || fy < 0 || fy >= height) {
                    std::cout << "Error! Agent '" << agent_id << "' has coordinates out of bounds." << std::endl;
                    return false;
                }
                agents_tasks->emplace_back(sy, sx, fy, fx);
            }
            ////////////////////////////////////////////////////////////////////////////////
            start_i = (*agents_tasks)[0].start_i;
            start_j = (*agents_tasks)[0].start_j;
            goal_i = (*agents_tasks)[0].goal_i;
            goal_j = (*agents_tasks)[0].goal_j;
            /////////////////////////////////////////////////////////////////////////////////
        }
        else if (value == CNS_TAG_GRID) {
            hasGrid = true;
            if (!(hasHeight && hasWidth)) {
                std::cout << "Error! No '" << CNS_TAG_WIDTH << "' tag or '" << CNS_TAG_HEIGHT << "' tag before '"
                          << CNS_TAG_GRID << "'tag encountered!" << std::endl;
                return false;
            }
            nlohmann::ordered_json grid = element.value();
            while (grid_i < height) {
                if (grid[grid_i].empty()) {
                    std::cout << "Error! Not enough '" << CNS_TAG_ROW << "' tags inside '" << CNS_TAG_GRID << "' tag."
                              << std::endl;
                    std::cout << "Number of '" << CNS_TAG_ROW
                              << "' tags should be equal (or greater) than the value of '" << CNS_TAG_HEIGHT
                              << "' tag which is " << height << std::endl;
                    return false;
                }
                grid_j = 0;
                nlohmann::json cur = json::parse(std::string(grid[grid_i]));
                if (cur.size() == width)
                    for (grid_j = 0; grid_j < width; ++grid_j) {
                        Grid[grid_i][grid_j] = cur[grid_j];
                    }

                if (grid_j != width) {
                    std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << grid_i + 1 << " " << CNS_TAG_ROW
                              << std::endl;
                    return false;
                }
                ++grid_i;
            }
        }
    }
    //some additional checks
    if (!hasGrid) {
        std::cout << "Error! There is no tag 'grid' in json-file!\n";
        return false;
    }

    if (Grid[start_i][start_j] != CN_GC_NOOBS) {
        std::cout << "Error! Start cell is not traversable (cell's value is" << Grid[start_i][start_j] << ")!"
                  << std::endl;
        return false;
    }

    if (Grid[goal_i][goal_j] != CN_GC_NOOBS) {
        std::cout << "Error! Goal cell is not traversable (cell's value is" << Grid[goal_i][goal_j] << ")!"
                  << std::endl;
        return false;
    }

    return true;
}

int Map::getValue(int i, int j) const
{
    if (i < 0 || i >= height)
        return -1;

    if (j < 0 || j >= width)
        return -1;

    return Grid[i][j];
}