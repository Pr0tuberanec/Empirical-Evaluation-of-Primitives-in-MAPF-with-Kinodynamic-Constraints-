#ifndef NODE_H
#define NODE_H

//That's the data structure for storing a single search node.
//You MUST store all the intermediate computations occuring during the search
//incapsulated to Nodes (so NO separate arrays of g-values etc.)

struct Node
{
    int     i, j; //grid cell coordinates
    double  F, g, H; //f-, g- and h-values of the search node
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)

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
        return F > v.F;
    }

    virtual Node* getParent() const {
        return parent;
    }
};

struct TimeNode : public Node
{
    int     start_t, end_t, t;
    TimeNode    *parent;

    TimeNode(){}

    TimeNode(int cur_i, int cur_j, double cur_g, double cur_H, double hweight, 
        int l_bnd_t, int r_bnd_t, int tm, TimeNode* cur_parent = nullptr)
        : Node(cur_i, cur_j, cur_g, cur_H, hweight) {
        start_t = l_bnd_t;
        end_t = r_bnd_t;
        t = tm;
        parent = cur_parent;
    }


    ~TimeNode() {}

    bool operator<(const TimeNode& v)
    {
        if (g + H == v.g + v.H) {
            return t > v.t;
        }
        return g + H > v.g + v.H;
    }

    virtual TimeNode* getParent() const override {
        return parent;
    }
};
#endif
