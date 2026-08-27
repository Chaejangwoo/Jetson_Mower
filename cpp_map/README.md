# GPS 지도 기반 Coverage Path Planning (CPP) 모듈

이 폴더는 GPS로 받은 잔디밭 지도를 격자 지도(grid map)로 바꾸고, 예초기가 이동할 기본 커버리지 경로를 계산하는 독립 모듈입니다.

현재 프로젝트의 전체 흐름은 다음과 같습니다.

```text
GPS 지도 범위
    ↓
20cm 단위 격자 지도
    ↓
예초기 7×7 안전 영역 검사
    ↓
CPP 경로 계산
    ↓
예초기 이동 명령
```

## 1. 지도 크기 계산

셀 하나를 `20cm × 20cm`, 즉 `0.2m × 0.2m`로 사용합니다.

따라서 지도에 100×100개의 셀이 있으면 실제 크기는 다음과 같습니다.

```text
가로: 100 × 0.2m = 20m
세로: 100 × 0.2m = 20m
결과: 약 20m × 20m
```

GPS 경계가 20cm로 정확히 나누어지지 않으면 북쪽과 동쪽의 남는 부분은 버립니다. 예를 들어 실제 가로 길이가 20.18m이면 100셀(20m)만 사용합니다.

## 2. 예초기 영역

예초기 전체 크기는 `1.4m × 1.4m`로 가정합니다.

```text
1.4m ÷ 0.2m = 7셀
```

따라서 예초기 중심을 기준으로 7×7 셀을 안전 할당 영역으로 사용합니다. 실제 절삭 영역은 장애물 충돌을 피하기 위해 중앙 3×3 셀만 사용합니다.

```text
0000000  ← 예초기 안전 할당 영역
0000000
0011100  ← 실제 절삭 영역
0011100
0011100
0000000
0000000
```

- `0`: 예초기가 차지하는 안전 영역
- `1`: 실제로 깎는 영역
- `#`: 장애물
- `.`: 이동 가능한 빈 공간
- `?`: 아직 정보가 없는 셀

예초기 중심 GPS가 셀로 변환되었을 때 7×7 영역 전체가 지도 안에 있어야 하고, 장애물과 겹치지 않아야 배치할 수 있습니다.

## 3. 현재 CPP 알고리즘

`coveragePath()`는 가장 단순한 CPP 기준선 알고리즘인 lawn-mower/boustrophedon 방식을 사용합니다.

```text
→ → → → → → →
              ↓
← ← ← ← ← ← ←
↓
→ → → → → → →
```

실제 절삭 영역이 3×3이므로 예초기 중심점을 3셀 간격으로 배치하고, 각 행을 번갈아 왕복합니다. 예초기 7×7 영역이 지도 밖으로 나가거나 장애물과 겹치는 중심점은 제외합니다.

중요: 현재 버전은 장애물을 발견했을 때 반대편으로 안전하게 우회해 연결하는 기능까지는 구현하지 않았습니다. 따라서 장애물 주변 경로가 끊길 수 있으며, 실제 주행 전에는 장애물 팽창(inflation), 자유 공간 구간 분해, 우회 연결이 추가되어야 합니다.

## 4. 파일 구성

```text
cpp_map/
├── CMakeLists.txt                   # 빌드 설정
├── include/mower_map/grid_map.hpp   # 공개 API
├── src/grid_map.cpp                 # 격자/GPS/CPP 구현
├── examples/demo.cpp                # 7×7 영역 예제
├── examples/coverage_demo.cpp       # 100×100 지도 CPP 예제
├── examples/visualize_coverage.py   # Python 시각화
└── tests/grid_map_test.cpp          # 자동 테스트
```

## 5. 빌드 및 테스트

프로젝트 루트(`Jetson_Mower`)에서 실행합니다.

```bash
cmake -S cpp_map -B cpp_map/build
cmake --build cpp_map/build
ctest --test-dir cpp_map/build --output-on-failure
```

## 6. CPP 경로 시각화

100×100 셀 지도와 중앙 장애물을 대상으로 CPP 경로를 계산하고 PNG로 저장합니다.

```bash
cmake --build cpp_map/build --target mower_coverage_demo
python3 cpp_map/examples/visualize_coverage.py \
  --binary cpp_map/build/mower_coverage_demo \
  --output /tmp/mower_coverage.png
```

시각화에서 파란 선은 CPP 왕복 경로, 빨간색은 장애물, 파란 사각형은 현재 예초기의 7×7 안전 영역, 주황색은 실제 절삭 3×3 영역입니다.
