#include "kdtree.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

// ==================== 생성자/소멸자 ====================

KDTree::KDTree(const std::vector<Point3D> &pts) : root(nullptr)
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

    // 트리 구축
    root = build_tree(indices, 0);
}

KDTree::~KDTree()
{
    destroy_tree(root);
}

void KDTree::destroy_tree(KDNode *node)
{
    if (!node)
        return;
    destroy_tree(node->left);
    destroy_tree(node->right);
    delete node;
}

// ==================== 거리 계산 ====================

float KDTree::distance(const Point3D &a, const Point3D &b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ==================== 트리 구축 ====================

KDNode *KDTree::build_tree(std::vector<int> &indices, int depth)
{
    if (indices.empty())
        return nullptr;

    // 축 선택 (x=0, y=1, z=2 순환)
    int axis = depth % 3;

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

    // 중앙값 선택
    size_t median = indices.size() / 2;
    KDNode *node = new KDNode(indices[median]);

    // 좌우 서브트리 재귀 구축
    std::vector<int> left_indices(indices.begin(), indices.begin() + median);
    std::vector<int> right_indices(indices.begin() + median + 1, indices.end());

    node->left = build_tree(left_indices, depth + 1);
    node->right = build_tree(right_indices, depth + 1);

    return node;
}

// ==================== 반경 탐색 ====================

void KDTree::search_radius(KDNode *node, const Point3D &target, float radius,
                           std::vector<int> &neighbors, int depth)
{
    if (!node)
        return;

    // 현재 노드와의 거리
    float dist = distance(points[node->index], target);

    if (dist <= radius)
    {
        neighbors.push_back(node->index);
    }

    // 축 선택
    int axis = depth % 3;
    float target_val, node_val;

    if (axis == 0)
    {
        target_val = target.x;
        node_val = points[node->index].x;
    }
    else if (axis == 1)
    {
        target_val = target.y;
        node_val = points[node->index].y;
    }
    else
    {
        target_val = target.z;
        node_val = points[node->index].z;
    }

    // 가까운 쪽 먼저
    KDNode *near = (target_val < node_val) ? node->left : node->right;
    KDNode *far = (target_val < node_val) ? node->right : node->left;

    search_radius(near, target, radius, neighbors, depth + 1);

    // 반대편도 확인 필요한지
    float axis_dist = std::abs(target_val - node_val);
    if (axis_dist <= radius)
    {
        search_radius(far, target, radius, neighbors, depth + 1);
    }
}
std::vector<int> KDTree::find_radius(const Point3D &target, float radius)
{
    std::vector<int> neighbors;
    search_radius(root, target, radius, neighbors, 0);
    return neighbors;
}