cmake_minimum_required(VERSION 3.25)

# 프로젝트 설정
set(PROJECT_NAME program)
set(CMAKE_CXX_STANDARD 17)

project(${PROJECT_NAME})

# ========== ExternalProject 설정 ==========
include(ExternalProject)

# Dependency 관련 변수 설정
set(DEP_INSTALL_DIR ${PROJECT_BINARY_DIR}/external)
set(DEP_INCLUDE_DIR ${DEP_INSTALL_DIR}/include)
set(DEP_LIB_DIR ${DEP_INSTALL_DIR}/lib)

# ========== GLFW ==========
ExternalProject_Add(
    dep_glfw
    GIT_REPOSITORY "https://github.com/glfw/glfw.git"
    GIT_TAG "3.3.8"
    GIT_SHALLOW 1
    UPDATE_COMMAND "" 
    PATCH_COMMAND "" 
    TEST_COMMAND ""
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
        -DGLFW_BUILD_EXAMPLES=OFF
        -DGLFW_BUILD_TESTS=OFF
        -DGLFW_BUILD_DOCS=OFF
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
)
set(DEP_LIST ${DEP_LIST} dep_glfw)
set(DEP_LIBS ${DEP_LIBS} glfw3)

# ========== GLAD ==========
ExternalProject_Add(
    dep_glad
    GIT_REPOSITORY "https://github.com/Dav1dde/glad"
    GIT_TAG "v0.1.36"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
        -DGLAD_INSTALL=ON
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
    TEST_COMMAND ""
)
set(DEP_LIST ${DEP_LIST} dep_glad)
set(DEP_LIBS ${DEP_LIBS} glad)

# ========== GLM ==========
ExternalProject_Add(
    dep_glm
    GIT_REPOSITORY "https://github.com/g-truc/glm"
    GIT_TAG "0.9.9.8"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    TEST_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PROJECT_BINARY_DIR}/dep_glm-prefix/src/dep_glm/glm
        ${DEP_INSTALL_DIR}/include/glm
)
set(DEP_LIST ${DEP_LIST} dep_glm)

# ========== Native File Dialog ==========
ExternalProject_Add(
    dep_nfd
    GIT_REPOSITORY "https://github.com/btzy/nativefiledialog-extended.git"
    GIT_TAG "master"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}
        -DNFD_BUILD_TESTS=OFF
    TEST_COMMAND ""
)
set(DEP_LIST ${DEP_LIST} dep_nfd)
set(DEP_LIBS ${DEP_LIBS} nfd)


# ========== ImGui ==========
add_library(imgui
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_impl_glfw.cpp
    imgui/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PRIVATE ${DEP_INCLUDE_DIR})
add_dependencies(imgui ${DEP_LIST})
set(DEP_INCLUDE_DIR ${DEP_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(DEP_LIST ${DEP_LIST} imgui)
set(DEP_LIBS ${DEP_LIBS} imgui)

# ========== 소스 파일 수집 ==========
file(GLOB SOURCES "src/*.cpp")

# ========== 실행 파일 생성 ==========
add_executable(${PROJECT_NAME} ${SOURCES})

# ========== 인클루드 디렉토리 ==========
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${DEP_INCLUDE_DIR}
)

# ========== 라이브러리 디렉토리 ==========
target_link_directories(${PROJECT_NAME} PUBLIC 
    ${DEP_LIB_DIR}
)

# ========== 라이브러리 링크 ==========
target_link_libraries(${PROJECT_NAME} PUBLIC 
    ${DEP_LIBS}
    opengl32
    gdi32
)

# ========== Dependency 추가 ==========
add_dependencies(${PROJECT_NAME} ${DEP_LIST})

# ========== 정적 링크 옵션 ==========
target_link_options(${PROJECT_NAME} PRIVATE 
    -static
    -static-libgcc
    -static-libstdc++
)