#include "Node.h"

bool operator<(const Node& v1, const Node& v2) {
    if (v1.t_lower + v1.h != v2.t_lower + v2.h) {
        return v1.t_lower + v1.h < v2.t_lower + v2.h;
    }

    if (v1.h != v2.h) {
        return v1.h < v2.h;
    }

    if (v1.t_upper != v2.t_upper) {
        return v1.t_upper > v2.t_upper;
    }

    if (v1.i != v2.i) {
        return v1.i < v2.i;
    }

    if (v1.j != v2.j) {
        return v1.j < v2.j;
    }

    if (v1.dir != v2.dir) {
        return v1.dir < v2.dir;
    }

    if (v1.vel != v2.vel) {
        return v1.vel < v2.vel;
    }

    return false;
}