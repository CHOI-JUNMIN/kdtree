#ifndef RTREE_H
#define RTREE_H

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "point3d.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Boost 3D 포인트 타입
typedef bg::model::point<float, 3, bg::cs::cartesian> boost_point_t;
typedef std::pair<boost_point_t, int> value_t; // (포인트, 인덱스)

// R*-tree 클래스 (Boost 기반)
class RTree
{
private:
    std::vector<Point3D> points;
    bgi::rtree<value_t, bgi::rstar<16>> tree; // R*-tree, max_entries=16

public:
    RTree(const std::vector<Point3D> &pts, int max_entries = 16);
    ~RTree();

    std::vector<int> find_radius(const Point3D &target, float radius);
};

#endif // RTREE_H