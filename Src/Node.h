#ifndef NODE_H
#define NODE_H

#include "Primitive.h"

struct Node {
    int i = 0;
    int j = 0;
    int vel = 0;
    int dir = 0;

    int t_lower = 0;
    int t_upper = 0;

    Primitive primitive;
    double h = 0;
    int parent = -1;

    explicit Node() = default;

    explicit Node(int i, int j, int primitive_end_vel, int t_lower, int t_upper)
        : i(i), j(j), vel(primitive_end_vel), t_lower(t_lower), t_upper(t_upper) {}

    explicit Node(int i, int j, int dir, int vel, int t_lower, int t_upper,
         Primitive primitive, double h = 0, int parent = -1)
        : i(i), j(j), dir(dir), vel(vel),
          t_lower(t_lower), t_upper(t_upper),
          primitive(primitive), h(h), parent(parent) {}

    friend bool operator<(const Node& v1, const Node& v2);
};

#endif