// src/obj_loader.h
#ifndef OBJ_loader_H
#define OBJ_loader_H

#include <vector>
#include <string>

// ========== 구조체 정의 ==========

// 정점 (위치 + 색상)
struct Vertex
{
    float x, y, z;  // 위치
    float r, g, b;  // 색상 (0.0 ~ 1.0)
    bool has_color; // 색상 정보 유무
};

// 텍스처 좌표
struct TexCoord
{
    float u, v;
};

// 법선
struct Normal
{
    float x, y, z;
};

// 면 (삼각형)
struct Face
{
    int v[3];  // 정점 인덱스 (0-based)
    int vt[3]; // 텍스처 인덱스 (-1 = 없음)
    int vn[3]; // 법선 인덱스 (-1 = 없음)
};

// OBJ 메시
struct OBJMesh
{
    std::vector<Vertex> vertices;
    std::vector<TexCoord> texcoords;
    std::vector<Normal> normals;
    std::vector<Face> faces;
    bool has_vertex_colors;

    // 생성자
    OBJMesh() : has_vertex_colors(false) {}
};

// ========== 함수 선언 ==========

// OBJ 파일 로드
OBJMesh *load_obj(const std::string &filename);

// 메모리 해제
void free_mesh(OBJMesh *mesh);

void save_filtered_mesh(const OBJMesh *mesh, const std::vector<bool> &is_noise,
                        const std::string &output_path);

#endif // OBJ_loader_H