#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include "point3d.h"

// KD-Tree 노드
struct KDNode
{
    int index; // 원본 정점 인덱스
    KDNode *left;
    KDNode *right;

    KDNode(int idx) : index(idx), left(nullptr), right(nullptr) {}
};

// KD-Tree 클래스
class KDTree
{
private:
    KDNode *root;
    std::vector<Point3D> points;

    // 재귀적으로 트리 구축
    KDNode *build_tree(std::vector<int> &indices, int depth);

    void search_radius(KDNode *node, const Point3D &target, float radius,
                       std::vector<int> &neighbors, int depth);

    // 거리 계산
    float distance(const Point3D &a, const Point3D &b);

    // 메모리 해제
    void destroy_tree(KDNode *node);

public:
    KDTree(const std::vector<Point3D> &pts);
    ~KDTree();

    std::vector<int> find_radius(const Point3D &target, float radius);

};

#endif // KDTREE_H