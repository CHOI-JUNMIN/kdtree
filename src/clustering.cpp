#include "clustering.h"
#include <map>
#include <queue>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>

std::vector<ClusterInfo> analyze_clusters(
    const std::vector<Point3D> &points,
    const std::vector<int> &labels)
{
    std::map<int, std::vector<int>> cluster_map;

    // 클러스터별로 점 분류
    for (size_t i = 0; i < labels.size(); i++)
    {
        cluster_map[labels[i]].push_back(i);
    }

    std::vector<ClusterInfo> clusters;

    for (auto &pair : cluster_map)
    {
        int id = pair.first;
        std::vector<int> &indices = pair.second;

        ClusterInfo info;
        info.id = id;
        info.size = indices.size();

        // 중심 계산
        Point3D sum(0, 0, 0);
        for (int idx : indices)
        {
            sum.x += points[idx].x;
            sum.y += points[idx].y;
            sum.z += points[idx].z;
        }
        info.center.x = sum.x / indices.size();
        info.center.y = sum.y / indices.size();
        info.center.z = sum.z / indices.size();

        // 반경 계산
        float max_dist = 0;
        for (int idx : indices)
        {
            float dx = points[idx].x - info.center.x;
            float dy = points[idx].y - info.center.y;
            float dz = points[idx].z - info.center.z;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > max_dist)
                max_dist = dist;
        }
        info.radius = max_dist;

        clusters.push_back(info);
    }

    // 크기 순 정렬
    std::sort(clusters.begin(), clusters.end(),
              [](const ClusterInfo &a, const ClusterInfo &b)
              {
                  return a.size > b.size;
              });

    return clusters;
}

std::vector<int> dbscan_clustering_kdtree(
    const std::vector<Point3D> &points,
    KDTree &tree,
    float radius,
    int min_points)
{
    int n = points.size();
    std::vector<int> labels(n, -2); // -2: 미방문, -1: 노이즈, 0~: 클러스터
    int cluster_id = 0;

    std::cout << "DBSCAN 클러스터링 시작..." << std::endl;

    for (int i = 0; i < n; i++)
    {
        if (labels[i] != -2)
            continue; // 이미 방문

        // 이웃 찾기
        std::vector<int> neighbors = tree.find_radius(points[i], radius);

        if ((int)neighbors.size() < min_points)
        {
            labels[i] = -1; // 노이즈
            continue;
        }

        // 새 클러스터 시작
        labels[i] = cluster_id;

        // 클러스터 확장 (BFS)
        std::queue<int> to_expand;
        for (int neighbor : neighbors)
        {
            if (neighbor != i)
            {
                to_expand.push(neighbor);
            }
        }

        while (!to_expand.empty())
        {
            int current = to_expand.front();
            to_expand.pop();

            // 노이즈였던 점을 클러스터에 포함
            if (labels[current] == -1)
            {
                labels[current] = cluster_id;
            }

            if (labels[current] != -2)
                continue; // 이미 처리됨

            labels[current] = cluster_id;

            // current의 이웃도 찾기
            std::vector<int> current_neighbors = tree.find_radius(points[current], radius);

            // 밀집 지역이면 이웃들도 확장
            if ((int)current_neighbors.size() >= min_points)
            {
                for (int neighbor : current_neighbors)
                {
                    if (labels[neighbor] == -2 || labels[neighbor] == -1)
                    {
                        to_expand.push(neighbor);
                    }
                }
            }
        }

        cluster_id++;

        // 진행 상황
        if (i % 100000 == 0)
        {
            std::cout << "  진행: " << i << " / " << n
                      << " (클러스터: " << cluster_id << "개)" << std::endl;
        }
    }

    std::cout << "DBSCAN 완료! 총 " << cluster_id << "개 클러스터" << std::endl;

    return labels;
}