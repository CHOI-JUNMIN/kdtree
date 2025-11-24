#include "rtree_str.h"
#include <cmath>
#include <iostream>

// ==================== 생성자/소멸자 ====================

RTreeSTR::RTreeSTR(const std::vector<Point3D> &pts, int max_entries)
    : root(nullptr), max_entries(max_entries)
{
    points = pts;

    if (points.empty())
        return;

    std::vector<int> indices(points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        indices[i] = i;
    }

    root = build_tree_str(indices, 0);
}

RTreeSTR::~RTreeSTR()
{
    destroy_tree(root);
}

void RTreeSTR::destroy_tree(RNodeSTR *node)
{
    if (!node)
        return;
    delete node;
}

// ==================== 거리 계산 ====================

float RTreeSTR::distance(const Point3D &a, const Point3D &b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ==================== STR Bulk Loading ====================

RNodeSTR *RTreeSTR::build_tree_str(std::vector<int> &indices, int depth)
{
    if (indices.empty())
        return nullptr;

    if ((int)indices.size() <= max_entries)
    {
        RNodeSTR *leaf = new RNodeSTR(true);
        leaf->indices = indices;

        for (int idx : indices)
        {
            leaf->mbr.expand(points[idx]);
        }

        return leaf;
    }

    int axis = depth % 3;

    std::sort(indices.begin(), indices.end(),
              [this, axis](int a, int b)
              {
                  if (axis == 0)
                      return points[a].x < points[b].x;
                  if (axis == 1)
                      return points[a].y < points[b].y;
                  return points[a].z < points[b].z;
              });

    int num_slices = std::max(1, (int)std::ceil(std::sqrt((double)indices.size() / max_entries)));
    int slice_size = (indices.size() + num_slices - 1) / num_slices;

    RNodeSTR *internal = new RNodeSTR(false);

    for (int i = 0; i < num_slices; i++)
    {
        int start = i * slice_size;
        int end = std::min((int)indices.size(), (i + 1) * slice_size);

        if (start >= end)
            break;

        std::vector<int> slice_indices(indices.begin() + start, indices.begin() + end);
        RNodeSTR *child = build_tree_str(slice_indices, depth + 1);

        if (child)
        {
            internal->children.push_back(child);
            internal->mbr.expand(child->mbr);
        }
    }

    return internal;
}

// ==================== 반경 탐색 ====================

void RTreeSTR::search_radius(RNodeSTR *node, const Point3D &target, float radius,
                             std::vector<int> &neighbors)
{
    if (!node)
        return;

    if (!node->mbr.intersects_sphere(target, radius))
        return;

    if (node->is_leaf)
    {
        for (int idx : node->indices)
        {
            float dist = distance(points[idx], target);
            if (dist <= radius)
            {
                neighbors.push_back(idx);
            }
        }
    }
    else
    {
        for (RNodeSTR *child : node->children)
        {
            search_radius(child, target, radius, neighbors);
        }
    }
}

std::vector<int> RTreeSTR::find_radius(const Point3D &target, float radius)
{
    std::vector<int> neighbors;
    search_radius(root, target, radius, neighbors);
    return neighbors;
}