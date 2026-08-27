# GPS 기반 20cm 격자 지도 모듈

`mower_map`은 GPS 지도 범위를 20cm 셀로 바꾸고 예초기 작업영역을 보수적으로 표시하는 독립 C++17 모듈입니다.

## 기준

- 셀 1개: `20cm x 20cm`
- 예초기 할당 영역: `1.4m x 1.4m = 7 x 7` 셀
- 실제 절삭 영역: 중앙 `60cm x 60cm = 3 x 3` 셀
- GPS 원점: 지도 남서쪽(south-west)
- GPS 경계에서 20cm로 나누어지지 않는 북쪽/동쪽 남는 부분: 버림(`floor`)

행(row)은 남쪽에서 북쪽으로 증가합니다. `render()` 출력은 보기 편하도록 북쪽 행부터 반환합니다. 출력 기호는 `0=할당`, `1=절삭`, `#=장애물`, `.=빈 공간`, `?=미확인`입니다.

## 빌드 및 테스트

```bash
cmake -S cpp_map -B cpp_map/build
cmake --build cpp_map/build
ctest --test-dir cpp_map/build --output-on-failure
./cpp_map/build/mower_map_demo
```

GPS 위치는 예초기 중심의 위치로 넣습니다. `canPlaceMower()`는 7x7 전체가 지도 안에 있고 장애물과 겹치지 않는지 검사하며, `placeMower()`가 성공하면 7x7을 할당하고 중앙 3x3을 절삭 완료로 표시합니다.

## 간단한 커버리지 경로

`coveragePath()`는 장애물과 겹치지 않는 예초기 중심점을 3셀 간격으로 만들고, 각 행을 번갈아 왕복하는 단순 지그재그 경로를 반환합니다. 100×100 예시와 Python 시각화는 다음처럼 실행합니다.

```bash
cmake -S cpp_map -B cpp_map/build
cmake --build cpp_map/build --target mower_coverage_demo
python3 cpp_map/examples/visualize_coverage.py \
  --binary cpp_map/build/mower_coverage_demo \
  --output /tmp/mower_coverage.png
```

이 경로는 기준선(baseline) 알고리즘이며, 실제 적용 시에는 장애물 주변 우회와 GPS 오차 여유를 추가해야 합니다.
