#ifndef NODE_H
#define NODE_H
#include <vector>
#include "map.h"

//That's the data structure for storing a single search node.
//You MUST store all the intermediate computations occuring during the search
//incapsulated to Nodes (so NO separate arrays of g-values etc.)

struct Node
{
    int     i, j; //grid cell coordinates
    double  F = 0, g = 0, H = 0; //f-, g- and h-values of the search node
    Node    *parent = nullptr; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)

    Node(){}

    Node(int inp_i, int inp_j, double inp_g, double inp_H,
         double inp_hweight=1, Node* inp_parent=nullptr) {
        i = inp_i;
        j = inp_j;
        g = inp_g;
        H = inp_H;
        F = g + inp_hweight * H;
        parent = inp_parent;
    }

    ~Node() {}

    bool operator>(const Node& v) const
    {
        return g + H > v.g + v.H;
    }

    virtual Node* getParent() const {
        return parent;
    }
};

struct TimeNode : public Node
{
    int     start_t, end_t, t = 0;
    int     vel;
    TimeNode    *parent = nullptr;

    TimeNode(){}

    TimeNode(int cur_i, int cur_j, double cur_g, double cur_H, double hweight, 
        int l_bnd_t, int r_bnd_t, int tm, int cur_vel ,TimeNode* cur_parent = nullptr)
        : Node(cur_i, cur_j, cur_g, cur_H, hweight) {
        start_t = l_bnd_t;
        end_t = r_bnd_t;
        t = tm;
        vel = cur_vel;
        parent = cur_parent;
    }

    ~TimeNode() {}

    bool operator<(const TimeNode& v)
    {
        // if (g + H == v.g + v.H) {
        //     return t > v.t;
        // }
        return g + H > v.g + v.H;
        // if (g == v.g) {
        //     return H > v.H;
        // }
        // return g > v.g;
    }

    virtual TimeNode* getParent() const override {
        return parent;
    }
};

struct Cell {
    int i;
    int j;
    int nodeIdx;
    std::pair<int, int> intrvl;

    Cell(int i, int j, int idx, std::pair<int, int> t_bnd) : i(i), j(j), nodeIdx(idx), intrvl(t_bnd) {}
};

struct Primitive {
    int cost;
    int end_vel;
    std::vector<Cell> cells;

    Primitive(TimeNode curNode, std::pair<int, int> side_move, Map* map) {
        auto [di, dj] = side_move;
        Cell first(curNode.i, curNode.j, map->width * curNode.i + curNode.j, {0, 0});
        Cell second(curNode.i + di, curNode.j + dj, map->width * (curNode.i + di) + (curNode.j + dj), {1, 1});
        cells.push_back(first);
        cells.push_back(second);

        cost = 1;
        end_vel = 0;
    }
};
#endif
