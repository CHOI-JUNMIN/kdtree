#ifndef RTREE_STR_H
#define RTREE_STR_H

#include <vector>
#include <algorithm>
#include <limits>
#include "point3d.h"

struct MBR_STR
{
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;

    MBR_STR()
        : min_x(std::numeric_limits<float>::max()),
          min_y(std::numeric_limits<float>::max()),
          min_z(std::numeric_limits<float>::max()),
          max_x(std::numeric_limits<float>::lowest()),
          max_y(std::numeric_limits<float>::lowest()),
          max_z(std::numeric_limits<float>::lowest()) {}

    void expand(const Point3D &p)
    {
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        min_z = std::min(min_z, p.z);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
        max_z = std::max(max_z, p.z);
    }

    void expand(const MBR_STR &other)
    {
        min_x = std::min(min_x, other.min_x);
        min_y = std::min(min_y, other.min_y);
        min_z = std::min(min_z, other.min_z);
        max_x = std::max(max_x, other.max_x);
        max_y = std::max(max_y, other.max_y);
        max_z = std::max(max_z, other.max_z);
    }

    bool intersects_sphere(const Point3D &center, float radius) const
    {
        float dist_sq = 0.0f;

        if (center.x < min_x)
            dist_sq += (min_x - center.x) * (min_x - center.x);
        else if (center.x > max_x)
            dist_sq += (center.x - max_x) * (center.x - max_x);

        if (center.y < min_y)
            dist_sq += (min_y - center.y) * (min_y - center.y);
        else if (center.y > max_y)
            dist_sq += (center.y - max_y) * (center.y - max_y);

        if (center.z < min_z)
            dist_sq += (min_z - center.z) * (min_z - center.z);
        else if (center.z > max_z)
            dist_sq += (center.z - max_z) * (center.z - max_z);

        return dist_sq <= radius * radius;
    }
};

struct RNodeSTR
{
    MBR_STR mbr;
    bool is_leaf;
    std::vector<int> indices;
    std::vector<RNodeSTR *> children;

    RNodeSTR(bool leaf = true) : is_leaf(leaf) {}

    ~RNodeSTR()
    {
        for (auto child : children)
        {
            delete child;
        }
    }
};

class RTreeSTR
{
private:
    RNodeSTR *root;
    std::vector<Point3D> points;
    int max_entries;

    float distance(const Point3D &a, const Point3D &b);
    RNodeSTR *build_tree_str(std::vector<int> &indices, int depth);
    void search_radius(RNodeSTR *node, const Point3D &target, float radius,
                       std::vector<int> &neighbors);
    void destroy_tree(RNodeSTR *node);

public:
    RTreeSTR(const std::vector<Point3D> &pts, int max_entries = 100);
    ~RTreeSTR();

    std::vector<int> find_radius(const Point3D &target, float radius);
};

#endif