#ifndef QUADTREE_H_
#define QUADTREE_H_

#include <vector>
#include <memory>
#include <cmath>
#include <utility>
#include <ros/ros.h>

struct Point {
    double x, y;
    Point(double _x, double _y) : x(_x), y(_y) {}
};

struct ObjectData {
    ros::Time last_seen;
    float x, y;
};

class Quadtree {
private:
    struct Node {
        double x, y, width, height;
        std::vector<std::pair<Point, ObjectData>> points;
        std::unique_ptr<Node> nw = nullptr;
        std::unique_ptr<Node> ne = nullptr;
        std::unique_ptr<Node> sw = nullptr;
        std::unique_ptr<Node> se = nullptr;
        Node(double _x, double _y, double _width, double _height) 
            : x(_x), y(_y), width(_width), height(_height) {}
    };

    std::unique_ptr<Node> root;
    const size_t capacity;
    const size_t max_depth;

    void subdivide(Node* node) {
        double halfWidth = node->width / 2.0;
        double halfHeight = node->height / 2.0;
        node->nw = std::make_unique<Node>(node->x, node->y, halfWidth, halfHeight);
        node->ne = std::make_unique<Node>(node->x + halfWidth, node->y, halfWidth, halfHeight);
        node->sw = std::make_unique<Node>(node->x, node->y + halfHeight, halfWidth, halfHeight);
        node->se = std::make_unique<Node>(node->x + halfWidth, node->y + halfHeight, halfWidth, halfHeight);
    }

    bool insert(Node* node, const Point& point, const ObjectData& data, size_t depth) {
        if (depth >= max_depth) {
            node->points.push_back(std::make_pair(point, data));
            return true;
        }

        if (node->points.size() < capacity) {
            node->points.push_back(std::make_pair(point, data));
            return true;
        }

        if (node->nw == nullptr) {
            subdivide(node);
        }

        if (insert(node->nw.get(), point, data, depth + 1)) return true;
        if (insert(node->ne.get(), point, data, depth + 1)) return true;
        if (insert(node->sw.get(), point, data, depth + 1)) return true;
        if (insert(node->se.get(), point, data, depth + 1)) return true;

        return false;
    }

public:
    Quadtree(double x, double y, double width, double height, size_t _capacity, size_t _max_depth) 
        : root(std::make_unique<Node>(x, y, width, height)), capacity(_capacity), max_depth(_max_depth) {}

    bool insert(const Point& point, const ObjectData& data) {
        return insert(root.get(), point, data, 0);
    }

    void query(double x, double y, double range, std::vector<std::pair<Point, ObjectData>>& found) const {
        query(root.get(), x, y, range, found);
    }

    void query(const Node* node, double x, double y, double range, std::vector<std::pair<Point, ObjectData>>& found) const {
        if (node == nullptr) return;

        double halfWidth = node->width / 2.0;
        double halfHeight = node->height / 2.0;
        bool inRange = (x + range > node->x - halfWidth) && (x - range < node->x + halfWidth) &&
                       (y + range > node->y - halfHeight) && (y - range < node->y + halfHeight);
        
        if (!inRange) return;

        for (const auto& point : node->points) {
            double distance = std::hypot(point.first.x - x, point.first.y - y);
            if (distance <= range) {
                found.push_back(point);
            }
        }

        query(node->nw.get(), x, y, range, found);
        query(node->ne.get(), x, y, range, found);
        query(node->sw.get(), x, y, range, found);
        query(node->se.get(), x, y, range, found);
    }

    void clear() {
        root = std::make_unique<Node>(root->x, root->y, root->width, root->height);
    }
};

#endif // QUADTREE_H_
