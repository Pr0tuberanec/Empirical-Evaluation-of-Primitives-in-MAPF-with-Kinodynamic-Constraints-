#include "Heap.h"

BinHeap::BinHeap() {
    data.emplace_back();
}

BinHeap::BinHeap(int width) : map_width(width) {
    data.emplace_back();
}

void BinHeap::add(Node node) {
    ++size;
    data.push_back(node);
    position_lookup[node.i * map_width + node.j][node.t_lower] = size;
    siftUp(size);
}

void BinHeap::extractMin() {
    if (size == 0) {
        return;
    }
    
    int idx = data[1].i * map_width + data[1].j;
    position_lookup[idx].erase(data[1].t_lower);
    if (position_lookup[idx].empty()) {
        position_lookup.erase(idx);
    }

    if (size == 1) {
        data.pop_back();
        size--;
        return;
    }

    std::swap(data[1], data[size]);
    data.pop_back();
    position_lookup[data[1].i * map_width + data[1].j][data[1].t_lower] = 1;
    size--;
    siftDown(1);
}

void BinHeap::updateKey(const Node& old_node, const Node& new_node) {
    data[position_lookup[old_node.i * map_width + old_node.j][old_node.t_lower]] = new_node;
    siftUp(position_lookup[old_node.i * map_width + old_node.j][old_node.t_lower]);
}

void BinHeap::siftUp(int idx) {
    size_t parent_idx = idx / 2;
    if (parent_idx == 0) {
        return;
    }

    if (comparator(data[parent_idx], data[idx])) {
        std::swap(data[parent_idx], data[idx]);
        std::swap(position_lookup[data[parent_idx].i * map_width + data[parent_idx].j][data[parent_idx].t_lower],
                  position_lookup[data[idx].i * map_width + data[idx].j][data[idx].t_lower]);
        siftUp(parent_idx);
    }
}

void BinHeap::siftDown(int idx) {
    size_t child_idx = 2 * idx;
    if (child_idx > size) {
        return;
    }
    if ((child_idx + 1 <= size) && (comparator(data[child_idx], data[child_idx + 1]))) {
        child_idx++;
    }

    if (comparator(data[idx], data[child_idx])) {
        std::swap(data[child_idx], data[idx]);
        std::swap(position_lookup[data[child_idx].i * map_width + data[child_idx].j][data[child_idx].t_lower],
                  position_lookup[data[idx].i * map_width + data[idx].j][data[idx].t_lower]);
        siftDown(child_idx);
    }
}

Node BinHeap::getMin() const { return data[1]; }
size_t BinHeap::getSize() const { return size; }