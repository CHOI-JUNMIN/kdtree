#include "rtree.h"
#include <cmath>
#include <iostream>


RTree::RTree(const std::vector<Point3D> &pts, int max_entries)
{
    points = pts;

    std::vector<value_t> values;
    values.reserve(points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        boost_point_t p(points[i].x, points[i].y, points[i].z);
        values.push_back(std::make_pair(p, i));
    }

    tree = bgi::rtree<value_t, bgi::rstar<16>>(values); 
}
RTree::~RTree()
{
    // Boost가 자동으로 메모리 관리
}

std::vector<int> RTree::find_radius(const Point3D &target, float radius)
{
    std::vector<int> neighbors;

    bg::model::box<boost_point_t> query_bbox(
        boost_point_t(target.x - radius, target.y - radius, target.z - radius),
        boost_point_t(target.x + radius, target.y + radius, target.z + radius));

    std::vector<value_t> candidates;
    tree.query(
        bgi::intersects(query_bbox), // ← MBR
        std::back_inserter(candidates));

    float radius_sq = radius * radius; // 제곱 미리 계산 (sqrt 생략)

    for (const auto &v : candidates)
    {
        float dx = bg::get<0>(v.first) - target.x;
        float dy = bg::get<1>(v.first) - target.y;
        float dz = bg::get<2>(v.first) - target.z;
        float dist_sq = dx * dx + dy * dy + dz * dz;

        if (dist_sq <= radius_sq) // 구 내부 체크
        {
            neighbors.push_back(v.second);
        }
    }

    return neighbors;
}