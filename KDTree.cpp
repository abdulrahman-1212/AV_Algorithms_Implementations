#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

struct Point {
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

struct KDTreeNode {
    Point point;
    KDTreeNode* right;
    KDTreeNode* left;

    KDTreeNode(Point p) : point(p), right(nullptr), left(nullptr) {}
};

class KDTree {



private:
    KDTreeNode* root;
};



// #include <iostream>
// #include <vector>
// #include <algorithm>
// #include <cmath>

// using namespace std;

// // Define a 2D point structure
// struct Point {
//     double x, y;
    
//     Point(double x, double y) : x(x), y(y) {}
// };

// // Define a structure for the KD-Tree node
// struct KDTreeNode {
//     Point point;         // Data point
//     KDTreeNode* left;    // Left child (smaller value)
//     KDTreeNode* right;   // Right child (larger value)
    
//     KDTreeNode(Point p) : point(p), left(nullptr), right(nullptr) {}
// };

// // Class representing the KD-Tree
// class KDTree {
// private:
//     KDTreeNode* root;

//     // Function to insert a point into the KD-Tree
//     KDTreeNode* insert(KDTreeNode* node, Point point, int depth) {
//         if (node == nullptr) {
//             return new KDTreeNode(point);
//         }

//         // Calculate current dimension (alternating x and y)
//         int currentDim = depth % 2;

//         if (currentDim == 0) { // Compare x-coordinates
//             if (point.x < node->point.x) {
//                 node->left = insert(node->left, point, depth + 1);
//             } else {
//                 node->right = insert(node->right, point, depth + 1);
//             }
//         } else { // Compare y-coordinates
//             if (point.y < node->point.y) {
//                 node->left = insert(node->left, point, depth + 1);
//             } else {
//                 node->right = insert(node->right, point, depth + 1);
//             }
//         }

//         return node;
//     }

//     // Function to compute the squared distance between two points
//     double squaredDistance(Point p1, Point p2) {
//         return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
//     }

//     // Function to find the nearest neighbor
//     void nearestNeighbor(KDTreeNode* node, Point target, int depth, KDTreeNode*& best, double& bestDist) {
//         if (node == nullptr) {
//             return;
//         }

//         // Calculate squared distance between current node and target
//         double dist = squaredDistance(node->point, target);
//         if (dist < bestDist) {
//             bestDist = dist;
//             best = node;
//         }

//         // Calculate current dimension (alternating x and y)
//         int currentDim = depth % 2;

//         // Recursively search in the appropriate subtree
//         if ((currentDim == 0 && target.x < node->point.x) || (currentDim == 1 && target.y < node->point.y)) {
//             nearestNeighbor(node->left, target, depth + 1, best, bestDist);
//             if (currentDim == 0 && (target.x + bestDist >= node->point.x)) {
//                 nearestNeighbor(node->right, target, depth + 1, best, bestDist);
//             }
//             if (currentDim == 1 && (target.y + bestDist >= node->point.y)) {
//                 nearestNeighbor(node->right, target, depth + 1, best, bestDist);
//             }
//         } else {
//             nearestNeighbor(node->right, target, depth + 1, best, bestDist);
//             if (currentDim == 0 && (target.x - bestDist <= node->point.x)) {
//                 nearestNeighbor(node->left, target, depth + 1, best, bestDist);
//             }
//             if (currentDim == 1 && (target.y - bestDist <= node->point.y)) {
//                 nearestNeighbor(node->left, target, depth + 1, best, bestDist);
//             }
//         }
//     }

// public:
//     // Constructor to initialize the KD-Tree with a null root
//     KDTree() : root(nullptr) {}

//     // Function to insert a point into the tree
//     void insert(Point point) {
//         root = insert(root, point, 0);
//     }

//     // Function to find the nearest neighbor to a given point
//     Point nearestNeighbor(Point target) {
//         KDTreeNode* best = nullptr;
//         double bestDist = std::numeric_limits<double>::max();
//         nearestNeighbor(root, target, 0, best, bestDist);
//         return best->point;
//     }
// };

// // Test the KD-Tree
// int main() {
//     KDTree tree;

//     // Insert points into the tree
//     tree.insert(Point(2, 3));
//     tree.insert(Point(4, 7));
//     tree.insert(Point(5, 4));
//     tree.insert(Point(9, 6));
//     tree.insert(Point(8, 1));
//     tree.insert(Point(7, 2));

//     // Find the nearest neighbor to a target point
//     Point target(3, 4);
//     Point nearest = tree.nearestNeighbor(target);
    
//     cout << "Nearest neighbor to (" << target.x << ", " << target.y << "): ("
//          << nearest.x << ", " << nearest.y << ")" << endl;

//     return 0;
// }
