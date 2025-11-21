#ifndef RTREE_H
#define RTREE_H

#include <vector>
#include <algorithm>
#include <limits>
#include "point3d.h"

struct MBR
{
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;

    MBR()
        : min_x(std::numeric_limits<float>::max()),
          min_y(std::numeric_limits<float>::max()),
          min_z(std::numeric_limits<float>::max()),
          max_x(std::numeric_limits<float>::lowest()),
          max_y(std::numeric_limits<float>::lowest()),
          max_z(std::numeric_limits<float>::lowest()) {}

    // 점을 포함하도록 확장
    void expand(const Point3D &p)
    {
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        min_z = std::min(min_z, p.z);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
        max_z = std::max(max_z, p.z);
    }

    // 다른 MBR을 포함하도록 확장
    void expand(const MBR &other)
    {
        min_x = std::min(min_x, other.min_x);
        min_y = std::min(min_y, other.min_y);
        min_z = std::min(min_z, other.min_z);
        max_x = std::max(max_x, other.max_x);
        max_y = std::max(max_y, other.max_y);
        max_z = std::max(max_z, other.max_z);
    }

    // MBR과 구(sphere)가 교차하는지
    bool intersects_sphere(const Point3D &center, float radius) const
    {
        // 구의 중심에서 MBR까지의 최단 거리 계산
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

// R-tree 노드
struct RNode
{
    MBR mbr;
    bool is_leaf;
    std::vector<int> indices;      // 리프: 포인트 인덱스들
    std::vector<RNode *> children; // 내부 노드: 자식 노드들

    RNode(bool leaf = true) : is_leaf(leaf) {}

    ~RNode()
    {
        for (auto child : children)
        {
            delete child;
        }
    }
};

struct MBRBox
{
    MBR mbr;
    int level; // 트리 깊이 (루트=0)
};



// R-tree 클래스
class RTree
{
private:
    RNode *root;
    std::vector<Point3D> points;
    int max_entries; // 노드당 최대 엔트리 수

    float distance(const Point3D &a, const Point3D &b);
    RNode *build_tree_str(std::vector<int> &indices, int depth);
    void search_radius(RNode *node, const Point3D &target, float radius,
                       std::vector<int> &neighbors);
    void destroy_tree(RNode *node);
    void collect_mbrs(RNode *node, int level, std::vector<MBRBox> &boxes);

public:
    RTree(const std::vector<Point3D> &pts, int max_entries = 100);
    ~RTree();

    std::vector<int> find_radius(const Point3D &target, float radius);
    std::vector<MBRBox> get_all_mbrs(); // 모든 MBR 수집
};




#endif // 