#ifndef MAP_H
#define MAP_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "json.hpp"

#include "Constants.h"

class Map {
public:

    Map();
    ~Map();
    Map(const Map& other) = default;

    bool getMap(const char* filename);

    bool cellOnGrid(int i, int j) const;
    bool cellIsObstacle(int i, int j) const;
    bool cellIsTraversable(int i, int j) const;

    int getValue(int i, int j) const;
    int getHeight() const;
    int getWidth() const;
    const std::shared_ptr<std::vector<std::vector<int>>> getGrid() const;

private:

    int height = 0;
    int width = 0;
    std::shared_ptr<std::vector<std::vector<int>>> grid;
};

#endif
