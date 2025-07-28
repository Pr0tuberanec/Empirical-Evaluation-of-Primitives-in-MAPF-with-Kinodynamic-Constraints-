#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <vector>

#include "Move.h"

struct Primitive {
    int cost = 0;
    int end_dir = 0;
    int end_vel = 0;
    std::vector<Move> moves;

    explicit Primitive() = default;
    explicit Primitive(std::vector<Move> move_sequence, int end_direction, int end_velocity)
        : moves(std::move(move_sequence)), end_dir(end_direction), end_vel(end_velocity) {}
};

#endif