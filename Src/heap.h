#ifndef HEAP_H
#define HEAP_H

#include "map.h"
#include "node.h"

#include <cstddef>
#include <vector>
#include <unordered_map>

class BinHeap {
public:
    // Добавляем фиктивный узел для удобства индексации с 1
    explicit BinHeap(int width) : map_width_(width) { data_.emplace_back(); }
    BinHeap() { data_.emplace_back(); }

    struct Comparator {
        bool operator()(const TimeNode& v1, const TimeNode& v2) const {
            return v1.t > v2.t;
        }
    };

    void Add(TimeNode node);
    void ExtractMin();
    Comparator GetComparator() const { return comp_; }
    const std::vector<TimeNode>& GetData() const { return data_; }
    const std::unordered_map<int, std::unordered_map<int, int>>& GetKey() const { return key_; }
    TimeNode GetMin() const { return data_[1]; }
    size_t GetSize() const { return size_; }
    void UpdateKey(TimeNode old_node, TimeNode new_node);

private:
    Comparator comp_;
    std::vector<TimeNode> data_;
    std::unordered_map<int, std::unordered_map<int, int>> key_;
    int map_width_;
    size_t size_ = 0;

    void SiftDown(int idx);
    void SiftUp(int idx);
};

#endif // HEAP_H
