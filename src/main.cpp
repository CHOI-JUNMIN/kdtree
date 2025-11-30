#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>
#include <nfd.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "obj_loader.h"
#include "kdtree.h"
#include "clustering.h"
#include "floor.h"

// ========== 전역 변수 ==========
int window_width = 1280;
int window_height = 720;
bool floor_removed = false;
std::vector<Point3D> dbscan_result_points;
float floor_removal_time = 0.0f;

// 카메라 변수
glm::vec3 camera_pos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 camera_front = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 camera_up = glm::vec3(0.0f, 1.0f, 0.0f);
float camera_yaw = -90.0f;
float camera_pitch = 0.0f;

// 마우스 변수
bool camera_control = false;
glm::vec2 prev_mouse_pos = glm::vec2(0.0f, 0.0f);

// 데이터 변수
std::vector<Point3D> original_points;
std::vector<Point3D> filtered_points;
OBJMesh *mesh = nullptr;
KDTree *tree = nullptr;
std::string current_obj_name = "";

// 파라미터
float epsilon = 0.05f;
int min_points = 10;
float point_size = 2.0f;

// 통계
int total_points = 0;
int removed_points = 0;
bool dbscan_applied = false;
float last_execution_time = 0.0f;
std::vector<int> current_labels;

// OpenGL 버퍼
GLuint vao = 0;
GLuint vbo = 0;
GLuint shader_program = 0;

// 바닥 시각화용 전역 변수
GLuint floor_vao = 0;
GLuint floor_vbo = 0;
std::vector<Point3D> floor_vis_points;
bool show_floor_vis = false;
float floor_ratio = 0.15f;

//바닥 자르기
float search_radius = 0.1f;
float mid_start = 0.10f;
float mid_end = 0.40f;
int min_points_above = 30;

// ========== 셰이더 소스 ==========
const char *vertex_shader_source = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float pointSize;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    gl_PointSize = pointSize;
}
)";

const char *fragment_shader_source = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 color;  // <- 추가

void main()
{
    FragColor = vec4(color, 1.0);
}
)";

// ========== 셰이더 컴파일 ==========
GLuint compile_shader(GLenum type, const char *source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(shader, 512, nullptr, info_log);
        std::cerr << "셰이더 컴파일 실패:\n"
                  << info_log << std::endl;
    }

    return shader;
}

// ========== 셰이더 프로그램 생성 ==========
void create_shader_program()
{
    GLuint vertex_shader = compile_shader(GL_VERTEX_SHADER, vertex_shader_source);
    GLuint fragment_shader = compile_shader(GL_FRAGMENT_SHADER, fragment_shader_source);

    shader_program = glCreateProgram();
    glAttachShader(shader_program, vertex_shader);
    glAttachShader(shader_program, fragment_shader);
    glLinkProgram(shader_program);

    int success;
    glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetProgramInfoLog(shader_program, 512, nullptr, info_log);
        std::cerr << "셰이더 링크 실패:\n"
                  << info_log << std::endl;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
}

// ========== OpenGL 버퍼 업데이트 ==========
void update_point_cloud_buffer(const std::vector<Point3D> &points)
{
    if (vao == 0)
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
    }

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(Point3D), points.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
void render_point_cloud()
{
    glUseProgram(shader_program);

    // 변환 행렬
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::lookAt(camera_pos, camera_pos + camera_front, camera_up);
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)window_width / window_height, 0.1f, 100.0f);

    glUniformMatrix4fv(glGetUniformLocation(shader_program, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shader_program, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shader_program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniform1f(glGetUniformLocation(shader_program, "pointSize"), point_size);

    // 1. 전체 포인트 (회색)
    glm::vec3 point_color(0.5f, 0.5f, 0.5f);
    glUniform3fv(glGetUniformLocation(shader_program, "color"), 1, glm::value_ptr(point_color));

    glBindVertexArray(vao);
    const std::vector<Point3D> &current_points = dbscan_applied ? filtered_points : original_points;
    glDrawArrays(GL_POINTS, 0, current_points.size());

    // 2. 바닥 포인트 (빨간색) - 덮어 그리기
    if (show_floor_vis && !floor_vis_points.empty())
    {
        glDisable(GL_DEPTH_TEST); // Z-fighting 방지

        glm::vec3 floor_color(1.0f, 0.0f, 0.0f);
        glUniform3fv(glGetUniformLocation(shader_program, "color"), 1, glm::value_ptr(floor_color));

        glBindVertexArray(floor_vao);
        glDrawArrays(GL_POINTS, 0, floor_vis_points.size());

        glEnable(GL_DEPTH_TEST); // 다시 활성화
    }

    glBindVertexArray(0);
}

// ========== DBSCAN 실행 ==========
void apply_dbscan()
{
    std::cout << "\nDBSCAN 실행 중..." << std::endl;
    std::cout << "Epsilon: " << epsilon << ", MinPts: " << min_points << std::endl;

    // 시간 측정 시작
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<int> labels = dbscan_clustering_kdtree(original_points, *tree, epsilon, min_points);
    current_labels = labels;

    // 시간 측정 종료
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    last_execution_time = duration.count() / 1000.0f;

    std::cout << "실행 시간: " << last_execution_time << " s" << std::endl;

    // 가장 큰 클러스터 찾기
    std::map<int, int> cluster_sizes;
    for (int label : labels)
    {
        cluster_sizes[label]++;
    }

    int largest_cluster = -1;
    int largest_size = 0;
    for (auto &[id, size] : cluster_sizes)
    {
        if (id != -1 && size > largest_size)
        {
            largest_cluster = id;
            largest_size = size;
        }
    }

    // 필터링된 포인트 생성
    filtered_points.clear();
    for (size_t i = 0; i < labels.size(); i++)
    {
        if (labels[i] == largest_cluster)
        {
            filtered_points.push_back(original_points[i]);
        }
    }

    removed_points = original_points.size() - filtered_points.size();
    dbscan_applied = true;

    // OpenGL 버퍼 업데이트
    update_point_cloud_buffer(filtered_points);
    dbscan_result_points = filtered_points;

    std::cout << "완료! 제거된 점: " << removed_points << std::endl;
}

// ========== 결과 저장 ==========
void save_result()
{
    if (!dbscan_applied)
    {
        std::cout << "먼저 DBSCAN을 실행하세요." << std::endl;
        return;
    }

    // 가장 큰 클러스터 찾기
    std::map<int, int> cluster_sizes;
    for (int label : current_labels)
    {
        cluster_sizes[label]++;
    }

    int largest_cluster = -1;
    int largest_size = 0;
    for (auto &[id, size] : cluster_sizes)
    {
        if (id != -1 && size > largest_size)
        {
            largest_cluster = id;
            largest_size = size;
        }
    }

    // labels로 직접 is_noise 생성
    std::vector<bool> is_noise(original_points.size(), false);
    for (size_t i = 0; i < current_labels.size(); i++)
    {
        if (current_labels[i] != largest_cluster)
        {
            is_noise[i] = true;
        }
    }

    // 저장
    char filename[100];
    sprintf(filename, "../model/dbscan_e%.3f_m%d.obj", epsilon, min_points);
    save_filtered_mesh(mesh, is_noise, filename);
}

// ========== 키 입력 처리 ==========
void process_input(GLFWwindow *window)
{
    if (!camera_control)
        return;

    const float camera_speed = 0.05f;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera_pos += camera_speed * camera_front;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera_pos -= camera_speed * camera_front;

    auto camera_right = glm::normalize(glm::cross(camera_up, -camera_front));
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera_pos += camera_speed * camera_right;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera_pos -= camera_speed * camera_right;

    auto camera_up_vec = glm::normalize(glm::cross(-camera_front, camera_right));
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera_pos += camera_speed * camera_up_vec;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        camera_pos -= camera_speed * camera_up_vec;
}

// ========== 마우스 콜백 ==========
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if (action == GLFW_PRESS)
        {
            ImGuiIO &io = ImGui::GetIO();
            if (!io.WantCaptureMouse)
            {
                double x, y;
                glfwGetCursorPos(window, &x, &y);
                prev_mouse_pos = glm::vec2((float)x, (float)y);
                camera_control = true;
            }
        }
        else if (action == GLFW_RELEASE)
        {
            camera_control = false;
        }
    }
}

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos)
{
    if (!camera_control)
        return;

    auto pos = glm::vec2((float)xpos, (float)ypos);
    auto delta_pos = pos - prev_mouse_pos;

    const float camera_rot_speed = 0.5f;
    camera_yaw += delta_pos.x * camera_rot_speed;
    camera_pitch -= delta_pos.y * camera_rot_speed;

    if (camera_yaw < 0.0f)
        camera_yaw += 360.0f;
    if (camera_yaw > 360.0f)
        camera_yaw -= 360.0f;
    if (camera_pitch > 89.0f)
        camera_pitch = 89.0f;
    if (camera_pitch < -89.0f)
        camera_pitch = -89.0f;

    // cameraFront 업데이트
    camera_front.x = cos(glm::radians(camera_yaw)) * cos(glm::radians(camera_pitch));
    camera_front.y = sin(glm::radians(camera_pitch));
    camera_front.z = sin(glm::radians(camera_yaw)) * cos(glm::radians(camera_pitch));
    camera_front = glm::normalize(camera_front);

    prev_mouse_pos = pos;
}

// ========== 원본으로 리셋 ==========
void reset_to_original()
{
    dbscan_applied = false;
    removed_points = 0;
    last_execution_time = 0.0f;
    show_floor_vis = false;
    floor_vis_points.clear();
    update_point_cloud_buffer(original_points);
}

// ========== 새 OBJ 파일 로드 ==========
void load_new_obj(const char *filepath)
{
    std::cout << "\n새 OBJ 파일 로딩 중: " << filepath << std::endl;

    std::string path_str(filepath);
    size_t last_slash = path_str.find_last_of("/\\");
    if (last_slash != std::string::npos)
    {
        current_obj_name = path_str.substr(last_slash + 1);
    }
    else
    {
        current_obj_name = path_str;
    }

    // 1. 기존 데이터 정리
    if (mesh)
    {
        free_mesh(mesh);
    }
    if (tree)
    {
        delete tree;
    }
    original_points.clear();
    filtered_points.clear();

    // 2. 새 OBJ 로드
    mesh = load_obj(filepath);
    if (!mesh)
    {
        std::cerr << "OBJ 로드 실패: " << filepath << std::endl;
        return;
    }

    // 3. Point cloud 생성
    for (const auto &v : mesh->vertices)
    {
        original_points.push_back(Point3D(v.x, v.y, v.z));
    }
    total_points = original_points.size();
    std::cout << "총 " << total_points << "개 포인트 로드" << std::endl;

    // 4. KD-Tree
    std::cout << "KD-Tree 구축 중..." << std::endl;
    tree = new KDTree(original_points);
    std::cout << "KD-Tree 구축 완료!" << std::endl;

    // 5. 상태 초기화
    dbscan_applied = false;
    removed_points = 0;
    last_execution_time = 0.0f;
    show_floor_vis = false;
    floor_vis_points.clear();

    // 6. OpenGL 버퍼 업데이트
    update_point_cloud_buffer(original_points);

    std::cout << "새 모델 로드 완료!" << std::endl;
}

// ========== 파일 선택 대화상자 ==========
void open_file_dialog()
{
    nfdchar_t *outPath = NULL;
    nfdfilteritem_t filters[1] = {{"OBJ Files", "obj"}};

    nfdresult_t result = NFD_OpenDialog(&outPath, filters, 1, NULL);

    if (result == NFD_OKAY)
    {
        load_new_obj(outPath);
        NFD_FreePath(outPath);
    }
    else if (result == NFD_CANCEL)
    {
        std::cout << "파일 선택 취소" << std::endl;
    }
    else
    {
        std::cerr << "에러: " << NFD_GetError() << std::endl;
    }
}

void apply_floor_removal()
{
    if (!dbscan_applied)
    {
        std::cout << "먼저 DBSCAN을 실행하세요." << std::endl;
        return;
    }

    std::cout << "\n=== 바닥 제거 시작 ===" << std::endl;

    int before_count = dbscan_result_points.size();

    // 시간 측정 시작
    auto start = std::chrono::high_resolution_clock::now();

    // 수직 기둥 보호 방식으로 바닥 제거
    filtered_points = remove_floor_with_column_protection(
        dbscan_result_points,
        floor_ratio,
        search_radius,
        mid_start,
        mid_end,
        min_points_above);

    // 시간 측정 끝
    auto end = std::chrono::high_resolution_clock::now();

    // 표시 업데이트
    total_points = before_count;                            // DBSCAN 후 점 개수
    removed_points = before_count - filtered_points.size(); // Floor 제거된 점
    last_execution_time = std::chrono::duration<float>(end - start).count();

    // OpenGL 버퍼 업데이트
    update_point_cloud_buffer(filtered_points);

    floor_removed = true;
    show_floor_vis = false;

    std::cout << "=== 바닥 제거 완료 ===" << std::endl;
}

void visualize_floor()
{
    if (!dbscan_applied)
    {
        std::cout << "먼저 DBSCAN을 실행하세요." << std::endl;
        return;
    }

    // 바닥 점들만 추출 (빨간색용)
    floor_vis_points = get_floor_points(dbscan_result_points, floor_ratio);

    // floor_vao 업데이트
    if (floor_vao == 0)
    {
        glGenVertexArrays(1, &floor_vao);
        glGenBuffers(1, &floor_vbo);
    }

    glBindVertexArray(floor_vao);
    glBindBuffer(GL_ARRAY_BUFFER, floor_vbo);
    glBufferData(GL_ARRAY_BUFFER, floor_vis_points.size() * sizeof(Point3D),
                floor_vis_points.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void *)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // 원본 vao는 그대로 유지 (건드리지 않음!)

    show_floor_vis = true;
}

int main()
{
    NFD_Init();

    // OBJ 파일 로드
    std::string obj_path = "../model/d314.obj";

    // 파일 이름 자동 추출
    size_t last_slash = obj_path.find_last_of("/\\");
    if (last_slash != std::string::npos)
    {
        current_obj_name = obj_path.substr(last_slash + 1);
    }
    else
    {
        current_obj_name = obj_path;
    }

    std::cout << "OBJ 파일 로딩 중..." << std::endl;
    mesh = load_obj(obj_path.c_str());
    if (!mesh)
    {
        std::cerr << "OBJ 로드 실패" << std::endl;
        return -1;
    }

    // Point cloud 생성
    for (const auto &v : mesh->vertices)
    {
        original_points.push_back(Point3D(v.x, v.y, v.z));
    }
    total_points = original_points.size();
    std::cout << "총 " << total_points << "개 포인트 로드" << std::endl;

    // KD-Tree 구축
    std::cout << "KD-Tree 구축 중..." << std::endl;
    tree = new KDTree(original_points);
    std::cout << "KD-Tree 구축 완료!" << std::endl;

    // GLFW 초기화
    if (!glfwInit())
    {
        std::cerr << "GLFW 초기화 실패" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(window_width, window_height, "DBSCAN Point Cloud Viewer", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "윈도우 생성 실패" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    // GLAD 초기화
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "GLAD 초기화 실패" << std::endl;
        return -1;
    }

    // ImGui 초기화
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // OpenGL 설정
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // 셰이더 생성
    create_shader_program();

    // 초기 버퍼 생성
    update_point_cloud_buffer(original_points);

    // 메인 루프
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // 키 입력 처리
        process_input(window);

        // ImGui 프레임 시작
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // GUI 패널
        ImGui::Begin("DBSCAN Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Separator();

        if (ImGui::Button("Load OBJ"))
        {
            open_file_dialog();
        }

        ImGui::SameLine();

        if (ImGui::Button("DBSCAN"))
        {
            apply_dbscan();
        }

        ImGui::SameLine();

        if (ImGui::Button("Remove Floor"))
        {
            apply_floor_removal();
        }

        ImGui::SameLine();

        if (ImGui::Button("Reset"))
        {
            reset_to_original();
        }

        ImGui::SameLine();

        if (ImGui::Button("Save Result"))
        {
            save_result();
        }

        ImGui::Separator();

        ImGui::Text("Current OBJ: %s", current_obj_name.c_str());
        ImGui::Text("Total Points: %d", total_points);

        if (dbscan_applied)
        {
            ImGui::Text("Removed Points: %d", removed_points);
            ImGui::Text("Remaining Points: %d", total_points - removed_points);
            ImGui::Text("Execution Time: %.2f s", last_execution_time);
        }

        ImGui::Separator();

        ImGui::PushItemWidth(250);
        ImGui::SliderFloat("Point Size", &point_size, 1.0f, 5.0f);
        ImGui::PopItemWidth();

        ImGui::Separator();

        ImGui::PushItemWidth(220);
        ImGui::InputFloat("Epsilon (Radius)", &epsilon, 0.001f, 0.01f, "%.3f");
        ImGui::InputInt("MinPts", &min_points);
        ImGui::PopItemWidth();

        ImGui::Separator();

        ImGui::PushItemWidth(250);
        ImGui::Text("Floor Visualization:");
        ImGui::SliderFloat("Floor Ratio", &floor_ratio, 0.05f, 0.30f, "%.2f");

        if (ImGui::Button("Show Floor"))
        {
            visualize_floor();
        }

        if (show_floor_vis)
        {
            ImGui::SameLine();
            if (ImGui::Button("Hide Floor"))
            {
                show_floor_vis = false;
            }
            ImGui::Text("Floor Points: %d", (int)floor_vis_points.size());
        }

        ImGui::Separator();
        ImGui::Text("Column Protection Settings:");
        ImGui::SliderFloat("Search Radius", &search_radius, 0.01f, 1.0f, "%.2f");
        ImGui::SliderFloat("Mid Start", &mid_start, 0.05f, 0.20f, "%.2f");
        ImGui::SliderFloat("Mid End", &mid_end, 0.20f, 0.60f, "%.2f");
        ImGui::SliderInt("Min Points", &min_points_above, 5, 500);

        ImGui::Separator();

        ImGui::Text("Controls (Right-click required):");
        ImGui::BulletText("Drag: Rotate Camera");
        ImGui::BulletText("W/A/S/D: Move");
        ImGui::BulletText("Q/E: Up/Down");

        ImGui::End();

        // 렌더링
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        render_point_cloud();

        // ImGui 렌더링
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // 정리
    delete tree;
    free_mesh(mesh);

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteProgram(shader_program);

    if (floor_vao != 0)
    {
        glDeleteVertexArrays(1, &floor_vao);
        glDeleteBuffers(1, &floor_vbo);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    NFD_Quit();

    return 0;
}