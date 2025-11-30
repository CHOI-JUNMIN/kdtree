#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include <cmath>
#include "point3d.h"
#include "kdtree.h"

// 평면 구조체: ax + by + cz + d = 0
struct Plane
{
    float a, b, c, d; // 평면 방정식 계수

    Plane() : a(0), b(0), c(1), d(0) {}
    Plane(float _a, float _b, float _c, float _d) : a(_a), b(_b), c(_c), d(_d) {}

    // 점과 평면 사이의 거리 계산
    float distance_to_point(const Point3D &p) const
    {
        return std::abs(a * p.x + b * p.y + c * p.z + d) /
               std::sqrt(a * a + b * b + c * c);
    }

    // 평면의 높이 (Y 좌표) 반환
    float get_y_height() const
    {
        if (std::abs(b) < 1e-6)
            return 0.0f;
        return -d / b;
    }
};

// 바닥 영역 점들만 추출 (시각화용)
std::vector<Point3D> get_floor_points(
    const std::vector<Point3D> &points,
    float floor_ratio = 0.15f);

std::vector<Point3D> remove_floor_with_column_protection(
    const std::vector<Point3D> &points,
    float floor_ratio = 0.15f,
    float search_radius = 0.1f,
    float mid_start = 0.10f,
    float mid_end = 0.40f,
    int min_points_above = 30);
    
#endif // RANSAC_H