/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
    :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
    : root(NULL)
    {}

    void insertHelper(Node *&node, uint depth, std::vector<float> point, int id)
    {
        if (node == NULL) // if traversing arrives a NULL node
        {
            node = new Node(point, id);
        }
        else { // traversing the node
            uint splitAxis;
            splitAxis = depth % 2;  // split x (even) or y (odd) region 
            (point[splitAxis] < node->point[splitAxis]) ? insertHelper(node->left, depth+1, point, id) : insertHelper(node->right, depth+1, point, id);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        insertHelper(root, 0, point, id);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        return ids;
    }
};




