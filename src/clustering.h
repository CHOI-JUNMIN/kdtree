#ifndef CLUSTERING_H
#define CLUSTERING_H

#include "kdtree.h"
#include "rtree.h"
#include "rtree_str.h"
#include <vector>
#include <string>

struct ClusterInfo
{
    int id;
    int size;
    Point3D center;
    float radius;
};

std::vector<int> dbscan_clustering_kdtree(
    const std::vector<Point3D> &points,
    KDTree &tree,
    float radius,
    int min_points);

std::vector<ClusterInfo> analyze_clusters(
    const std::vector<Point3D> &points,
    const std::vector<int> &labels);

std::vector<int> dbscan_clustering_rtree(
    const std::vector<Point3D> &points,
    RTree &tree,
    float radius,
    int min_points);

std::vector<int> dbscan_clustering_rtree_str(
    const std::vector<Point3D> &points,
    RTreeSTR &tree,
    float radius,
    int min_points);

#endif

