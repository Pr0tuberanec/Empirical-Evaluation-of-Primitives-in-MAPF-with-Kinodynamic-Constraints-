#include "heap.h"

void BinHeap::Add(TimeNode node) {
    ++size_;
    data_.push_back(node);
    key_[node.i * map_width_ + node.j][node.start_t] = size_;
    SiftUp(size_);
}

void BinHeap::ExtractMin() {
    if (size_ == 0) {
        return;
    } else if (size_ == 1) {
        key_[data_[1].i * map_width_ + data_[1].j].erase(data_[1].start_t);
        if (key_[data_[1].i * map_width_ + data_[1].j].empty()) {
            key_.erase(data_[1].i * map_width_ + data_[1].j);
        }
        data_.pop_back();
        size_--;
    } else {
        key_[data_[1].i * map_width_ + data_[1].j].erase(data_[1].start_t);
        if (key_[data_[1].i * map_width_ + data_[1].j].empty()) {
            key_.erase(data_[1].i * map_width_ + data_[1].j);
        }
        std::swap(data_[1], data_[size_]);
        data_.pop_back();
        key_[data_[1].i * map_width_ + data_[1].j][data_[1].start_t] = 1;
        size_--;
        SiftDown(1);
    }
}

void BinHeap::UpdateKey(TimeNode old_node, TimeNode new_node) {
    data_[key_[old_node.i * map_width_ + old_node.j][old_node.start_t]] = new_node;
    SiftUp(key_[old_node.i * map_width_ + old_node.j][old_node.start_t]);
}

void BinHeap::SiftUp(int idx) {
    size_t tmp = idx / 2;
    if (tmp == 0) {
        return;
    }
    if (comp_(data_[tmp], data_[idx])) {
        std::swap(data_[tmp], data_[idx]);
        std::swap(key_[data_[tmp].i * map_width_ + data_[tmp].j][data_[tmp].start_t],
                  key_[data_[idx].i * map_width_ + data_[idx].j][data_[idx].start_t]);
        SiftUp(tmp);
    }
}

void BinHeap::SiftDown(int idx) {
    size_t tmp = 2 * idx;
    if (tmp > size_) {
        return;
    }
    if ((tmp + 1 <= size_) && (comp_(data_[tmp], data_[tmp + 1]))) {
        tmp++;
    }
    if (comp_(data_[idx], data_[tmp])) {
        std::swap(data_[tmp], data_[idx]);
        std::swap(key_[data_[tmp].i * map_width_ + data_[tmp].j][data_[tmp].start_t],
                  key_[data_[idx].i * map_width_ + data_[idx].j][data_[idx].start_t]);
        SiftDown(tmp);
    }
}
