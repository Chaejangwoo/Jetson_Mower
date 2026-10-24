# mower_binary_grid

Intel RealSense D455의 **깊이 정보**와 기존 AFR 객체·영역 인식 결과를 합쳐,
예초기가 지나갈 수 있는 곳과 지나가면 안 되는 곳을 20cm 격자로 만드는
ROS 2 패키지입니다.

```text
격자 값 0 = 예초기가 통과할 수 있음
격자 값 1 = 장애물이 있거나 아직 안전하다고 확인하지 못함
```

## 왜 이 모듈이 필요한가

기존 `mower_vision_bridge`는 RGB 영상에서 객체 종류, 영상 속 사각형 위치,
화면 아래쪽을 8x10으로 나눈 영역 상태와 대표 깊이값을 제공합니다. 하지만
이 값만으로는 장애물이 예초기 기준으로 **전방 몇 m, 좌측 몇 m에 있는지**
알 수 없습니다.

이 패키지는 D455의 깊이 픽셀을 3차원 좌표로 바꾸고, 그 좌표를 20cm 지도
셀에 넣어 경로 계획기가 사용할 수 있는 이진 지도를 만듭니다.

## 전체 데이터 흐름

```text
D455 컬러 영상
      │
      ▼
mower_vision_bridge ── /vision/result ──┐
                                        │
D455 정렬 Depth ────────────────────────┼─> mower_binary_grid
D455 CameraInfo ────────────────────────┘          │
                                                   ├─> 이진 JSON 격자
                                                   └─> ROS OccupancyGrid
```

처리 순서는 다음과 같습니다.

1. `/vision/result`에서 AFR의 객체 탐지와 8x10 영역 상태를 받습니다.
2. 같은 시각의 정렬 Depth 프레임을 선택합니다.
3. `CameraInfo`로 깊이 픽셀을 3차원 점으로 바꿉니다.
4. 카메라 장착 위치와 각도로 점을 `base_link` 좌표로 변환합니다.
5. RGB 의미 정보와 점의 높이를 함께 사용해 장애물 여부를 판단합니다.
6. 예초기 전방 영역을 20cm 셀로 나눠 최종 0/1 지도를 발행합니다.

## 기본 지도 규격

| 항목 | 기본값 |
|---|---:|
| 셀 한 칸 | 0.20m x 0.20m |
| 전방 범위 | 0~6m |
| 좌우 범위 | 오른쪽 3m~왼쪽 3m |
| 지도 크기 | 30행 x 30열 |
| 기준 좌표계 | `base_link` |
| X축 | 예초기 전방 |
| Y축 | 예초기 왼쪽 |
| Z축 | 위쪽 |

이 지도는 현재 예초기 주변을 나타내는 **로컬 지도**입니다. GPS 꼭짓점으로
만드는 전체 작업 구역 지도와의 결합은 후속 단계입니다.

## 입력 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/vision/result` | `std_msgs/String` | AFR 객체 탐지와 8x10 상태 격자 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 컬러 영상에 정렬된 D455 깊이 영상 |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 3차원 역투영에 필요한 카메라 내부 파라미터 |

`/vision/result`와 Depth의 촬영 시각 차이가 기본 100ms를 넘으면 해당 결과는
사용하지 않습니다. 서로 다른 시점의 영상을 합쳐 잘못된 장애물 위치를
만드는 것을 방지하기 위한 조건입니다.

## 출력 토픽

### `/perception/local_binary_grid`

타입은 `std_msgs/String`이며 JSON이 들어 있습니다.

```json
{
  "frame_id": "base_link",
  "resolution_m": 0.2,
  "width": 30,
  "height": 30,
  "origin": {"forward_m": 0.0, "left_m": -3.0},
  "value_contract": {"free": 0, "blocked": 1},
  "free_cells": 140,
  "blocked_cells": 760,
  "data": [
    [1, 1, 1, 1],
    [1, 0, 0, 1],
    [1, 0, 1, 1]
  ]
}
```

`data`의 실제 크기는 기본 설정에서 30x30입니다. 위 예시는 형태를 보여주기
위해 일부만 표시했습니다.

### `/perception/local_occupancy_grid`

ROS 표준 `nav_msgs/OccupancyGrid`입니다.

| 의미 | OccupancyGrid 값 |
|---|---:|
| 통과 가능 | 0 |
| 통과 불가능 | 100 |

RViz 또는 다른 ROS 경로 계획 모듈에서 바로 사용할 수 있습니다.

## 장애물 판단 방식

깊이만 보고 `지면보다 높으면 무조건 장애물`로 처리하지 않습니다. 긴 풀도
깊이 카메라에서는 돌출된 물체처럼 보일 수 있기 때문입니다.

| AFR 판단 | 깊이·높이 판단 | 최종 결과 |
|---|---|---|
| `mow_now` | 낮고 풀로 볼 수 있는 형상 | 통과 가능 |
| `mow_now` | 허용 높이보다 큰 형상 | 통과 불가능 |
| `dangerous` | 무관 | 통과 불가능 |
| `deferred_mowable` | 무관 | 통과 불가능 |
| `unknown` | 무관 | 통과 불가능 |
| 객체 탐지 상자 | 하단 접촉부에 유효 깊이 존재 | 통과 불가능 |
| 깊이 미관측 | 안전 여부 확인 불가 | 통과 불가능 |

객체 탐지 상자 전체의 깊이를 사용하면 배경과 바닥까지 장애물로 잘못 칠할
수 있습니다. 따라서 상자의 **하단 25% 접촉 구간**만 지도 위치 계산에
사용합니다.

`mow_now` 영역은 유연한 풀을 고려해 기본 30cm까지 별도 허용치를 사용합니다.
이 값은 현장 시험 결과에 따라 반드시 조정해야 합니다.

## 빌드 및 실행

### 1. 빌드

```bash
cd ~/mower_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mower_binary_grid --symlink-install
source install/setup.bash
```

### 2. 선행 노드 확인

RealSense와 기존 비전 노드가 먼저 실행되어 있어야 합니다.

```bash
ros2 topic list | grep aligned_depth_to_color
ros2 topic list | grep /vision/result
```

### 3. 이진 격자 실행

```bash
ros2 launch mower_binary_grid binary_grid.launch.py
```

### 4. 출력 확인

```bash
ros2 topic echo --once /perception/local_binary_grid
ros2 topic echo --once /perception/local_occupancy_grid
```

## 주요 설정값

설정은 `launch/binary_grid.launch.py`에서 바꿀 수 있습니다.

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `resolution_m` | 0.20 | 셀 한 칸의 크기 |
| `forward_max_m` | 6.0 | 지도 전방 최대 범위 |
| `lateral_min_m` | -3.0 | 지도 오른쪽 끝 |
| `lateral_max_m` | 3.0 | 지도 왼쪽 끝 |
| `depth_min_m` | 0.30 | 사용할 최소 깊이 |
| `depth_max_m` | 6.0 | 사용할 최대 깊이 |
| `camera_forward_m` | 0.0 | 예초기 중심 기준 카메라 전방 위치 |
| `camera_left_m` | 0.0 | 예초기 중심 기준 카메라 좌측 위치 |
| `camera_height_m` | 0.60 | 지면 기준 카메라 높이 |
| `camera_pitch_down_deg` | 0.0 | 카메라 하향 각도 |
| `obstacle_min_height_m` | 0.12 | 일반 영역 장애물 최소 높이 |
| `mowable_grass_tolerance_m` | 0.30 | 예초 가능 풀 영역의 높이 허용치 |
| `detection_score_threshold` | 0.35 | 객체 탐지를 사용할 최소 신뢰도 |
| `detection_contact_band_fraction` | 0.25 | 객체 상자 하단에서 사용할 비율 |

### 장착 후 반드시 측정할 값

현재 아래 값은 개발용 임시값입니다.

```text
camera_forward_m
camera_left_m
camera_height_m
camera_pitch_down_deg
camera_yaw_left_deg
camera_roll_deg
```

실제 카메라 장착 후 렌즈 중심을 기준으로 측정해 수정해야 합니다. 이 값을
보정하지 않으면 장애물이 실제 지도 위치와 다른 셀에 표시될 수 있습니다.

## 시험

```bash
cd ~/mower_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/mower_binary_grid/test/test_grid_core.py
```

현재 단위시험은 다음을 확인합니다.

- 깊이값이 없으면 모든 셀이 통과 불가능인지
- 안전한 낮은 지면점까지 통과 가능한 경로가 생성되는지
- 객체 탐지가 풀 높이 허용 규칙보다 우선하는지
- `unknown` 의미 상태가 통과 불가능으로 처리되는지

## D455 실기기 확인 결과

2026-10-24 Jetson에서 실제 카메라와 비전 노드를 연결해 확인했습니다.

```text
지도 크기: 30x30
해상도: 0.20m
Depth-비전 시간 차이: 0.0ms
유효 깊이 표본: 6,761개
통과 가능 셀: 140개
통과 불가능 셀: 760개
출력값 종류: 0, 1
단위시험: 4개 통과
```

이 수치는 당시 카메라가 보고 있던 장면에 대한 예시이며 성능 보장값은
아닙니다.

## 현재 한계와 다음 단계

1. 카메라 실제 장착 높이·각도가 아직 확정되지 않았습니다.
2. 긴 풀 속에 가려진 돌처럼 RGB와 Depth가 모두 놓치는 장애물이 있을 수 있습니다.
3. 현재 출력은 단일 시점의 로컬 지도이며 여러 프레임을 누적하지 않습니다.
4. GPS 꼭짓점 다각형으로 만든 전역 작업 지도와 아직 결합되지 않았습니다.
5. 실제 주행 전 현장에서 풀 높이와 장애물 높이 기준을 보정해야 합니다.

권장 후속 순서는 다음과 같습니다.

```text
카메라 장착 치수 측정
→ 현장 데이터로 풀/장애물 기준 보정
→ 여러 프레임의 장애물 누적 및 제거 정책 추가
→ GPS 작업 다각형의 20cm 전역 지도와 결합
→ Coverage Path Planning에 OccupancyGrid 연결
```

