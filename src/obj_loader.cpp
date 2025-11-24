// src/obj_loader.cpp
#include "obj_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstring>

// ========== 보조 함수 ==========

// 면 한 줄 파싱 (삼각형 분할 포함)
void parse_face_line(const std::string &line, OBJMesh *mesh)
{
    std::istringstream iss(line);
    std::string token;
    iss >> token; // "f" 건너뛰기

    struct FaceVertex
    {
        int v, vt, vn;
        FaceVertex() : v(-1), vt(-1), vn(-1) {}
    };

    std::vector<FaceVertex> face_vertices;

    while (iss >> token)
    {
        FaceVertex fv;

        try
        { // ✅ 예외 처리 추가
            size_t first_slash = token.find('/');

            if (first_slash == std::string::npos)
            {
                // v만
                fv.v = std::stoi(token) - 1;
            }
            else
            {
                size_t second_slash = token.find('/', first_slash + 1);

                if (second_slash == std::string::npos)
                {
                    // v/vn
                    fv.v = std::stoi(token.substr(0, first_slash)) - 1;
                    fv.vn = std::stoi(token.substr(first_slash + 1)) - 1;
                }
                else
                {
                    // v/vt/vn
                    fv.v = std::stoi(token.substr(0, first_slash)) - 1;

                    std::string vt_str = token.substr(first_slash + 1, second_slash - first_slash - 1);
                    if (!vt_str.empty())
                    { // ✅ 빈 문자열 체크
                        fv.vt = std::stoi(vt_str) - 1;
                    }

                    fv.vn = std::stoi(token.substr(second_slash + 1)) - 1;
                }
            }

            face_vertices.push_back(fv);
        }
        catch (const std::exception &e)
        { // ✅ 예외 잡기
            // 파싱 실패한 면은 건너뜀
            std::cerr << "면 파싱 실패: " << token << std::endl;
            continue;
        }
    }

    // 삼각형 분할 (Fan Triangulation)
    // n개 정점 → (n-2)개 삼각형
    if (face_vertices.size() >= 3)
    {
        for (size_t i = 1; i < face_vertices.size() - 1; i++)
        {
            Face face;

            face.v[0] = face_vertices[0].v;
            face.v[1] = face_vertices[i].v;
            face.v[2] = face_vertices[i + 1].v;

            face.vt[0] = face_vertices[0].vt;
            face.vt[1] = face_vertices[i].vt;
            face.vt[2] = face_vertices[i + 1].vt;

            face.vn[0] = face_vertices[0].vn;
            face.vn[1] = face_vertices[i].vn;
            face.vn[2] = face_vertices[i + 1].vn;

            mesh->faces.push_back(face);
        }
    }
}

// ========== 메인 로드 함수 ==========

OBJMesh *load_obj(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << " 파일 열기 실패: " << filename << std::endl;
        return nullptr;
    }

    std::cout << "OBJ 파일 읽는 중: " << filename << std::endl;

    OBJMesh *mesh = new OBJMesh();

    std::string line;
    int line_num = 0;
    int polygon_count = 0; // 사각형 이상

    while (std::getline(file, line))
    {
        line_num++;

        // 빈 줄이나 주석 건너뛰기
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v")
        {
            // 정점 파싱
            Vertex v;
            iss >> v.x >> v.y >> v.z;

            // 색상 읽기 시도
            if (iss >> v.r >> v.g >> v.b)
            {
                v.has_color = true;
                mesh->has_vertex_colors = true;
            }
            else
            {
                v.has_color = false;
                v.r = v.g = v.b = 0.5f; // 기본 회색
            }

            mesh->vertices.push_back(v);
        }
        else if (type == "vt")
        {
            // 텍스처 좌표
            TexCoord tc;
            iss >> tc.u >> tc.v;
            mesh->texcoords.push_back(tc);
        }
        else if (type == "vn")
        {
            // 법선
            Normal n;
            iss >> n.x >> n.y >> n.z;
            mesh->normals.push_back(n);
        }
        else if (type == "f")
        {
            // 면 파싱
            int face_count_before = mesh->faces.size();
            parse_face_line(line, mesh);
            int triangles_added = mesh->faces.size() - face_count_before;

            if (triangles_added > 1)
            {
                polygon_count++;
            }
        }

        // 진행 상황 출력
        if (line_num % 100000 == 0)
        {
            std::cout << "  진행: " << line_num << " 줄" << std::endl;
        }
    }

    file.close();

    std::cout << "OBJ 파일 로드 완료!" << std::endl;
    std::cout << "   정점(v):    " << mesh->vertices.size();
    if (mesh->has_vertex_colors)
    {
        std::cout << " (색상 포함)";
    }
    std::cout << std::endl;
    std::cout << "   텍스처(vt): " << mesh->texcoords.size() << std::endl;
    std::cout << "   법선(vn):   " << mesh->normals.size() << std::endl;
    std::cout << "   면(f):      " << mesh->faces.size() << " (삼각형)" << std::endl;

    if (polygon_count > 0)
    {
        std::cout << "   ℹ" << polygon_count << "개의 다각형이 삼각형으로 분할됨" << std::endl;
    }

    return mesh;
}

void free_mesh(OBJMesh *mesh)
{
    delete mesh;
}

void save_filtered_mesh(const OBJMesh *mesh, const std::vector<bool> &is_noise,
                        const std::string &output_path)
{
    std::ofstream file(output_path);
    if (!file.is_open())
    {
        std::cerr << "파일 저장 실패: " << output_path << std::endl;
        return;
    }

    std::cout << "\n 필터링된 메시 저장 중..." << std::endl;

    // 새 정점 인덱스 매핑 (노이즈 제거 후)
    std::vector<int> new_index(mesh->vertices.size(), -1);
    int new_vertex_count = 0;

    // 1. 정상 정점만 저장
    for (size_t i = 0; i < mesh->vertices.size(); i++)
    {
        if (!is_noise[i])
        {
            const Vertex &v = mesh->vertices[i];

            // 색상 포함 여부에 따라
            if (v.has_color)
            {
                file << "v " << v.x << " " << v.y << " " << v.z << " "
                     << v.r << " " << v.g << " " << v.b << "\n";
            }
            else
            {
                file << "v " << v.x << " " << v.y << " " << v.z << "\n";
            }

            new_index[i] = new_vertex_count++;
        }

        if (i % 100000 == 0)
        {
            std::cout << "  정점: " << i << " / " << mesh->vertices.size() << std::endl;
        }
    }

    // 2. 법선 저장 (정상 정점의 법선만)
    for (size_t i = 0; i < mesh->normals.size(); i++)
    {
        if (i < mesh->vertices.size() && !is_noise[i])
        {
            const Normal &n = mesh->normals[i];
            file << "vn " << n.x << " " << n.y << " " << n.z << "\n";
        }

        if (i % 100000 == 0)
        {
            std::cout << "  법선: " << i << " / " << mesh->normals.size() << std::endl;
        }
    }

    // 3. 텍스처 좌표 저장 (있으면)
    for (size_t i = 0; i < mesh->texcoords.size(); i++)
    {
        if (i < mesh->vertices.size() && !is_noise[i])
        {
            const TexCoord &tc = mesh->texcoords[i];
            file << "vt " << tc.u << " " << tc.v << "\n";
        }

        if (i % 100000 == 0)
        {
            std::cout << "  텍스처: " << i << " / " << mesh->texcoords.size() << std::endl;
        }
    }

    // 4. 면 저장 (모든 정점이 정상인 면만)
    int valid_face_count = 0;
    //for (const Face &f : mesh->faces)
    for (size_t i = 0; i < mesh->faces.size(); i++)
    {
        const Face &f = mesh->faces[i];
        
        // 세 정점 모두 정상인지 확인
        if (f.v[0] >= 0 && f.v[0] < (int)is_noise.size() &&
            f.v[1] >= 0 && f.v[1] < (int)is_noise.size() &&
            f.v[2] >= 0 && f.v[2] < (int)is_noise.size() &&
            !is_noise[f.v[0]] && !is_noise[f.v[1]] && !is_noise[f.v[2]])
        {

            // 새 인덱스로 변환 (1-based)
            int v0 = new_index[f.v[0]] + 1;
            int v1 = new_index[f.v[1]] + 1;
            int v2 = new_index[f.v[2]] + 1;

            // 법선 인덱스도 변환
            if (f.vn[0] >= 0)
            {
                int vn0 = new_index[f.vn[0]] + 1;
                int vn1 = new_index[f.vn[1]] + 1;
                int vn2 = new_index[f.vn[2]] + 1;

                file << "f " << v0 << "//" << vn0 << " "
                     << v1 << "//" << vn1 << " "
                     << v2 << "//" << vn2 << "\n";
            }
            else
            {
                file << "f " << v0 << " " << v1 << " " << v2 << "\n";
            }

            valid_face_count++;
        }

        if (i % 100000 == 0)
        {
            std::cout << "  면: " << i << " / " << mesh->faces.size() << std::endl;
        }
    }

    file.close();

    std::cout << " 저장 완료: " << output_path << std::endl;
    std::cout << "   정점: " << new_vertex_count << " (원본: " << mesh->vertices.size() << ")" << std::endl;
    std::cout << "   면: " << valid_face_count << " (원본: " << mesh->faces.size() << ")" << std::endl;
}