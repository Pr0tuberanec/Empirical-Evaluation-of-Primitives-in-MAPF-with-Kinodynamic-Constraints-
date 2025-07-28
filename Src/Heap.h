#ifndef HEAP_H
#define HEAP_H

#include "Map.h"
#include "Node.h"

#include <cstddef>
#include <vector>
#include <unordered_map>

struct Comparator {
    bool operator()(const Node& v1, const Node& v2) const {
        if (v1.t_lower + v1.h != v2.t_lower + v2.h) {
            return v1.t_lower + v1.h > v2.t_lower + v2.h;
        }

        if (v1.h != v2.h) {
            return v1.h > v2.h;
        }

        if (v1.t_upper != v2.t_upper) {
            return v1.t_upper < v2.t_upper;
        }

        if (v1.i != v2.i) {
            return v1.i > v2.i;
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
};

class BinHeap {
public:

    // Добавляем фиктивный узел для удобства индексации с 1
    explicit BinHeap();
    explicit BinHeap(int width);

    void add(Node node);
    void extractMin();
    void updateKey(const Node& old_node, const Node& new_node);

    Node getMin() const;
    size_t getSize() const;

private:

    int map_width;
    size_t size = 0;
    Comparator comparator;
    std::vector<Node> data;

    std::unordered_map<int, std::unordered_map<int, int>> position_lookup;

    void siftDown(int idx);
    void siftUp(int idx);
};

#endif
