#include "floor.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>

std::vector<Point3D> get_floor_points(
    const std::vector<Point3D> &points,
    float floor_ratio)
{
    if (points.empty())
    {
        return std::vector<Point3D>();
    }

    // 범위 계산
    float y_min = points[0].y, y_max = points[0].y;
    float x_min = points[0].x, x_max = points[0].x;
    float z_min = points[0].z, z_max = points[0].z;
    for (const auto &p : points)
    {
        y_min = std::min(y_min, p.y);
        y_max = std::max(y_max, p.y);
        x_min = std::min(x_min, p.x);
        x_max = std::max(x_max, p.x);
        z_min = std::min(z_min, p.z);
        z_max = std::max(z_max, p.z);
    }

    float range = y_max - y_min;
    float floor_y_max = y_min + range * floor_ratio;

    std::cout << "\n바닥 영역 시각화:" << std::endl;
    std::cout << "  Y 범위: " << y_min << " ~ " << y_max << " (range: " << range << ")" << std::endl;
    std::cout << "  X 범위: " << x_min << " ~ " << x_max << " (range: " << x_max - x_min << ")" << std::endl;
    std::cout << "  Z 범위: " << z_min << " ~ " << z_max << " (range: " << z_max - z_min << ")" << std::endl;
    std::cout << "  바닥 영역 (하위 " << (floor_ratio * 100) << "%): "
              << y_min << " ~ " << floor_y_max << std::endl;

    // 바닥 영역 점들만 추출
    std::vector<Point3D> floor_points;
    for (const auto &p : points)
    {
        if (p.y <= floor_y_max)
        {
            floor_points.push_back(p);
        }
    }

    std::cout << "  바닥 점 개수: " << floor_points.size()
              << " / " << points.size()
              << " (" << (float)floor_points.size() / points.size() * 100.0f << "%)" << std::endl;

    return floor_points;
}

FloorRemovalResult remove_floor_with_column_protection(
    const std::vector<Point3D> &points,
    float floor_ratio,
    float search_radius,
    float mid_start,
    float mid_end,
    int min_points_above)
{
    FloorRemovalResult result;

    if (points.empty())
    {
        return result;
    }

    std::cout << "\n=== 수직 기둥 보호 바닥 제거 ===" << std::endl;

    // 1. Y 범위 계산
    float y_min = points[0].y, y_max = points[0].y;
    for (const auto &p : points)
    {
        y_min = std::min(y_min, p.y);
        y_max = std::max(y_max, p.y);
    }

    float range = y_max - y_min;
    float floor_y_max = y_min + range * floor_ratio;
    float mid_y_start = y_min + range * mid_start;
    float mid_y_end = y_min + range * mid_end;

    std::cout << "  Y 범위: " << y_min << " ~ " << y_max << " (range: " << range << ")" << std::endl;
    std::cout << "  바닥 영역 (하위 " << (floor_ratio * 100) << "%): Y < " << floor_y_max << std::endl;
    std::cout << "  중간 체크 영역: " << mid_y_start << " ~ " << mid_y_end << std::endl;
    std::cout << "  검색 반경 (XZ): " << search_radius << std::endl;
    std::cout << "  최소 점 개수: " << min_points_above << std::endl;

    // 2. 중간 높이 점들만 추출
    std::vector<Point3D> mid_points;
    for (const auto &p : points)
    {
        if (p.y >= mid_y_start && p.y <= mid_y_end)
        {
            mid_points.push_back(p);
        }
    }
    std::cout << "  중간 높이 점 개수: " << mid_points.size() << std::endl;

    // 3. 중간 높이 점들로 KD-Tree 생성
    KDTree mid_tree(mid_points);
    std::cout << "  중간 높이 KD-Tree 생성 완료" << std::endl;

    int floor_count = 0;
    int removed_count = 0;

    for (size_t i = 0; i < points.size(); i++)
    {
        const Point3D &p = points[i];

        // 바닥 영역이 아니면 무조건 유지
        if (p.y > floor_y_max)
        {
            result.filtered.push_back(p);
            continue;
        }

        floor_count++;

        // 4. 중간 높이 KD-Tree에서 XZ 반경으로 검색
        std::vector<int> neighbors = mid_tree.find_radius(p, search_radius);

        // 5. XZ 거리만 다시 체크 (Y는 이미 mid_points에서 필터링됨)
        int points_in_mid = 0;
        for (int idx : neighbors)
        {
            const Point3D &other = mid_points[idx];

            float dx = other.x - p.x;
            float dz = other.z - p.z;
            float xz_dist = std::sqrt(dx * dx + dz * dz);

            if (xz_dist <= search_radius)
            {
                points_in_mid++;
            }
        }

        // 중간 높이에 점이 충분히 많으면 유지 (기둥 아래)
        if (points_in_mid >= min_points_above)
        {
            result.filtered.push_back(p);
        }
        else
        {
            result.removed_indices.push_back(i); // 제거된 인덱스 기록
            removed_count++;
        }

        // 진행 상황
        if (i % 100000 == 0)
        {
            std::cout << "  진행: " << i << " / " << points.size() << std::endl;
        }
    }

    std::cout << "=== 완료 ===" << std::endl;
    std::cout << "  바닥 영역 점: " << floor_count << std::endl;
    std::cout << "  제거된 점 (노이즈): " << removed_count << std::endl;
    std::cout << "  남은 점: " << result.filtered.size() << std::endl;

    return result;
}