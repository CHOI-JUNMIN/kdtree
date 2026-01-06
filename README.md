![opengl](https://github.com/user-attachments/assets/53ee8363-74d6-4947-bd13-e6ee5229ff38)

## 포인트 클라우드 노이즈 제거 도구

C++ OpenGL기반의 프로그램으로 kdtree기반의 DBSCAN 노이즈 제거와 원통형 영역 기반의 노이즈제거에 초점을 둠

노이즈 제거를 위한 obj파일에 경우 폴더를 만들어서(ex: model) 그 안에 넣으면 됨

## 요구사항 (window)

* CMake 3.25 이상
* C++17 호환 컴파일러
* OpenGL 3.3 이상

## 주요 기능

* OBJ 로드 및 저장
* 노이즈 제거를 위한 DBSCAN
* point cloud 크기 조절
* 바닥영역 설정 파라미터
* 원통형 영역 기반의 노이즈제거 파라미터
* 실시간 3D 뷰어

## 주요 파라미터

### DBSCAN

![dbscan](https://github.com/user-attachments/assets/af0d8234-a303-4a28-96fc-6a85e260fb28)

- **Epsilon** : 이웃 탐색 반경
- **MinPts** : 최소 이웃 수 (자신 포함)


### 바닥 제거

![f123](https://github.com/user-attachments/assets/58b8849c-1e41-43d0-9b28-ddde69f6d774)




- **Floor Ratio** : 바닥 영역 범위 설정
- **Search Radius** : XZ 평면 탐색 반경
- **Mid Start** : 중간 높이 시작 지점
- **Mid End** : 중간 높이 끝 지점
- **Min Points ** : 중간 높이 영역 최소 점 개수
