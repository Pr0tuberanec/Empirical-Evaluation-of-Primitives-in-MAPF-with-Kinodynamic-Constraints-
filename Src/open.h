#ifndef HEAP_H
#define HEAP_H

#include <unordered_map>
#include <vector>
#include <cstddef>
#include "node.h"
#include "map.h"

class BinHeap {
public:
    // Добавляем фиктивный узел для удобства индексации с 1
    explicit BinHeap(int width) : map_width_(width) { data_.emplace_back(); }
    BinHeap() { data_.emplace_back(); }

    // struct Comparator {
    //     bool operator()(const Node& v1, const Node& v2) const {
    //         return v1.t > v2.t;
    //     }
    // };

    void Add(Node node);
    void ExtractMin();
    Comparator GetComparator() const { return comp_; }
    const std::vector<Node>& GetData() const { return data_; }
    const std::unordered_map<int, std::unordered_map<int, int>>& GetKey() const { return key_; }
    Node GetMin() const { return data_[1]; }
    size_t GetSize() const { return size_; }
    void UpdateKey(Node old_node, Node new_node);

private:
    Comparator comp_;
    std::vector<Node> data_;
    std::unordered_map<int, std::unordered_map<int, int>> key_;
    int map_width_;
    size_t size_ = 0;

    void SiftDown(int idx);
    void SiftUp(int idx);
};

#endif // HEAP_H
