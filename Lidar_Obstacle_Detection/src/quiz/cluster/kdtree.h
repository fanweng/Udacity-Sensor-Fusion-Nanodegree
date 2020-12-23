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
        if (node == NULL) { // if traversing arrives a NULL node
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

    // check if the node point is within a boxed square that is 2x distanceTol for length, centered around the target point
    bool isInBox(std::vector<float> target, std::vector<float> point, float distanceTol)
    {
        if ((point[0] < (target[0] - distanceTol)) || (point[0] > (target[0] + distanceTol)) || (point[1] < (target[1] - distanceTol)) || (point[1] > (target[1] + distanceTol))) {
            return false;
        }
        return true;
    }

    // calculate if the distance between node point and target point is within tolerance
    bool isInDistanceTol(std::vector<float> target, std::vector<float> point, float distanceTol)
    {
        float d = sqrtf(pow((target[0] - point[0]), 2.0) + pow((target[1] - point[1]), 2.0));

        if (d < distanceTol) {
            return true;
        }
        return false;
    }

    void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
    {
        if (node != NULL) {
            if (isInBox(target, node->point, distanceTol)) {
                if (isInDistanceTol(target, node->point, distanceTol)) {
                    // add point within distanceTol to the return list
                    ids.push_back(node->id);
                }
            }

            // branch to the next node depending on if the boxed square crosses over the divided x or y region
            if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) {
                searchHelper(target, node->left, depth+1, distanceTol, ids);
            }
            if ((target[depth % 2] + distanceTol) >= node->point[depth % 2]) {
                searchHelper(target, node->right, depth+1, distanceTol, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};




