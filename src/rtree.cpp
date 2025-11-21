#include "rtree.h"
#include <algorithm>
#include <cmath>
#include <iostream>

// ==================== 생성자/소멸자 ====================

RTree::RTree(const std::vector<Point3D> &pts, int max_entries)
    : root(nullptr), max_entries(max_entries)
{
    points = pts;

    if (points.empty())
        return;

    // 인덱스 배열 생성
    std::vector<int> indices(points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        indices[i] = i;
    }

    // STR (Sort-Tile-Recursive) bulk loading으로 트리 구축
    root = build_tree_str(indices, 0);
}

RTree::~RTree()
{
    destroy_tree(root);
}

void RTree::destroy_tree(RNode *node)
{
    if (!node)
        return;
    delete node;
}

// ==================== 거리 계산 ====================

float RTree::distance(const Point3D &a, const Point3D &b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ==================== STR Bulk Loading ====================

RNode *RTree::build_tree_str(std::vector<int> &indices, int depth)
{
    if (indices.empty())
        return nullptr;

    // 리프 노드 생성
    if ((int)indices.size() <= max_entries)
    {
        RNode *leaf = new RNode(true);
        leaf->indices = indices;

        // MBR 계산
        for (int idx : indices)
        {
            leaf->mbr.expand(points[idx]);
        }

        return leaf;
    }

    // 내부 노드 생성
    int axis = depth % 3; // 0=x, 1=y, 2=z

    // 해당 축으로 정렬
    std::sort(indices.begin(), indices.end(),
              [this, axis](int a, int b)
              {
                  if (axis == 0)
                      return points[a].x < points[b].x;
                  if (axis == 1)
                      return points[a].y < points[b].y;
                  return points[a].z < points[b].z;
              });

    // 슬라이스로 분할
    int num_slices = std::max(1, (int)std::ceil(std::sqrt((double)indices.size() / max_entries)));
    int slice_size = (indices.size() + num_slices - 1) / num_slices;

    RNode *internal = new RNode(false);

    for (int i = 0; i < num_slices; i++)
    {
        int start = i * slice_size;
        int end = std::min((int)indices.size(), (i + 1) * slice_size);

        if (start >= end)
            break;

        std::vector<int> slice_indices(indices.begin() + start, indices.begin() + end);
        RNode *child = build_tree_str(slice_indices, depth + 1);

        if (child)
        {
            internal->children.push_back(child);
            internal->mbr.expand(child->mbr);
        }
    }

    return internal;
}

// ==================== 반경 탐색 ====================

void RTree::search_radius(RNode *node, const Point3D &target, float radius,
                          std::vector<int> &neighbors)
{
    if (!node)
        return;

    // MBR이 검색 범위와 교차하지 않으면 스킵
    if (!node->mbr.intersects_sphere(target, radius))
        return;

    if (node->is_leaf)
    {
        // 리프 노드: 실제 거리 계산
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
        // 내부 노드: 자식들 탐색
        for (RNode *child : node->children)
        {
            search_radius(child, target, radius, neighbors);
        }
    }
}

std::vector<int> RTree::find_radius(const Point3D &target, float radius)
{
    std::vector<int> neighbors;
    search_radius(root, target, radius, neighbors);
    return neighbors;
}

std::vector<MBRBox> RTree::get_all_mbrs() // RTree:: 제거
{
    std::vector<MBRBox> boxes; // RTree:: 제거
    if (root)
    {
        collect_mbrs(root, 0, boxes);
    }
    return boxes;
}

void RTree::collect_mbrs(RNode *node, int level, std::vector<MBRBox> &boxes) // RTree:: 제거
{
    if (!node)
        return;

    MBRBox box; // RTree:: 제거
    box.mbr = node->mbr;
    box.level = level;
    boxes.push_back(box);

    if (!node->is_leaf)
    {
        for (RNode *child : node->children)
        {
            collect_mbrs(child, level + 1, boxes);
        }
    }
}