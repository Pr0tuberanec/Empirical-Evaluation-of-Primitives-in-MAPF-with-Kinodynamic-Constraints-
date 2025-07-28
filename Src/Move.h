#ifndef MOVE_H
#define MOVE_H

struct Move {
    int dy = 0;
    int dx = 0;
    int ft_t = 0; // first touch time
    int sw_t = 0; // sweeping time
    bool is_end_cell = false;

    explicit Move() = default;
    explicit Move(int dy, int dx, int ft_t, int sw_t, bool is_end_cell)
        : dy(dy), dx(dx), ft_t(ft_t), sw_t(sw_t), is_end_cell(is_end_cell) {}
};

#endif