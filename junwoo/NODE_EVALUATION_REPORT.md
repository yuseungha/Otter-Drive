# 📊 ROS 2 자율주행 노드 구조 정량적 평가 및 개선 보고서

---

## 📄 1. 개요 (Overview)

본 보고서는 현재 구축된 ROS 2 기반 자율주행 시스템(`ws_autonomy`)의 노드 구조, 데이터 흐름, 연산 효율성 및 제어 루프 완성도를 정량적으로 평가하고, 특히 **구형 비전 노드(`opencv_line_detect_node`) 구동으로 인한 성능 낭비 및 제어 단절 문제**를 상세히 분석하여 시스템 최적화 방안을 제시합니다.

---

## 🎯 2. 노드 구조 정량적 진단 요약

| 평가 항목 | 점수 | 진단 결과 요약 |
| :--- | :--- | :--- |
| **1. 제어 루프 연속성** | **0 / 100** | PID 제어 노드(`drive_node`) 미구동으로 `/cmd_vel` 0 Hz 발생 |
| **2. 비전 모듈 구조적 적합성** | **45 / 100** | **고도화 노드 대신 구형 Hough 기반 노드가 동작하여 제어 오차 신호 미발행** |
| **3. 연산 자원 및 HW 가속** | **35 / 100** | YOLO 노드가 Pure CPU로 동작 (CPU 637% 점유, 7.75 Hz 저조한 FPS) |
| **4. QoS 및 버퍼 정책** | **90 / 100** | YOLO 노드 `KEEP_LAST`, `depth=1` 적용으로 프레임 지연 방지 완료 |
| **종합 평점** | **42.5 / 100** | **[주의] 제어 루프 단절 및 구형 모듈 실행으로 자율주행 불능 상태** |

---

## 🔍 3. 비전 모듈 평가 점수(45/100) 세부 분석 (Deep-Dive)

현재 활성화되어 있는 `opencv_line_detect_node`와 휴면 상태인 고도화 노드 `computer_vision_node`를 정량적/알고리즘 측면에서 비교 분석한 결과입니다.

```mermaid
graph TD
    subgraph Current Execution (Score: 45)
        A[/image_raw] --> B[opencv_line_detect_node]
        B -->|Hough Lines| C[/opencv/line_detections - PoseArray]
        C -.->|조향 오차 계산 불가| D[drive_node - Broken!]
    end

    subgraph Recommended Architecture (Target: 95+)
        A[/image_raw] --> E[computer_vision_node]
        E -->|2차 다항식 & BEV| F[/lane_detection/steering_error - Float32]
        F -->|PID 제어| G[drive_node]
        G -->|/cmd_vel| H[Vehicle Actuator]
    end
```

### 3.1 항목별 정량 평가표 (Evaluation Matrix)

| 평가 세부 항목 | 비중 | 구형 `opencv_line_detect_node` | 고도화 `computer_vision_node` | 항목 점수 |
| :--- | :--- | :--- | :--- | :--- |
| **제어 오차 산출 능력** | 30% | ❌ 선분 좌표(`PoseArray`)만 발행 (오차 0) | ⭕ lateral, heading, steering_error 직접 산출 | **0 / 30 점** |
| **곡선 도로 대응력** | 25% | ❌ Hough 직선 근사 (곡선 이탈 및 노이즈) | ⭕ 2차 다항식(\(x = ay^2+by+c\)) 피팅 | **10 / 25 점** |
| **조명/색상 환경 적응력** | 20% | ⚠️ HSV 전용 (실내 조명/그림자 취약) | ⭕ HLS(흰색) + HSV(노란색) 이중 분할 | **10 / 20 점** |
| **HUD 디버그 호환성** | 15% | ⚠️ 비표준 토픽(`/opencv/*`) 발행 | ⭕ 표준 HUD 규격(`/lane_detection/*`) | **10 / 15 점** |
| **연산 및 통신 최적화** | 10% | ⚠️ Canny + Hough + 객체생성 낭비 | ⭕ NumPy Vectorized fast operations | **5 / 10 점** |
| **합계 점수** | **100%** | **현재 실행 중** | **미실행 (휴면)** | **45 / 100 점** |

---

### 3.2 핵심 감점 요인 상세 기술

#### 1. 제어 가능 신호(Control-Ready Signal) 미발행 (가장 결정적 감점 사유)
* **구형 노드 문제점**: `opencv_line_detect_node`는 검출된 차선 데이터를 단순히 2D 이미지 좌표계 상의 선분 시작점과 끝점(`PoseArray`) 형태로 퍼블리시합니다. 차량의 중심선 대비 편차(Lateral Error \(e_y\)), 도로 진행 방향과의 각도 편차(Heading Angle \(\Delta \psi\))를 전혀 계산하지 않으므로, 하위 차량 제어 노드가 이 데이터만으로는 차량 조향각을 계산할 수 없습니다.
* **고도화 노드 우수성**: `computer_vision_node`는 Lookahead 거리 시점(\(y_{\text{lookahead}}\))에서의 중앙 편차와 곡률 기울기 함수를 기반으로 하위 PID 제어기가 즉시 주입받을 수 있는 **1차원 정밀 조향 오차 신호(`steering_error`)**를 수학적으로 직접 계산하여 발행합니다.
  \[
  \text{Steering Error} = k_{\text{offset}} \cdot \frac{\text{Offset}}{W/2} + k_{\text{heading}} \cdot \arctan(2 a y + b)
  \]

#### 2. Hough 변환의 한계와 2차 다항식 곡선 피팅의 부재
* **구형 노드 문제점**: OpenCV의 `cv2.HoughLinesP` 알고리즘은 **직선(Line Segment)**만을 찾도록 설계되어 있어 곡선 구간에 진입하거나 노면 반사가 발생할 경우 차선을 오인식하거나 수많은 단편 선분으로 쪼개집니다.
* **고도화 노드 우수성**: `LaneDetector` 클래스는 좌/우 차선 픽셀에 대해 2차 다항식 방정식(\(x = a y^2 + b y + c\))을 피팅하여 연속적인 차선 곡선을 복원하므로, 완만한 곡선 및 급커브 환경에서도 안정적인 궤적 추종이 가능합니다.

#### 3. 색상 공간(Color Space) 분할의 환경 적응력 차이
* **구형 노드 문제점**: HSV 색상 공간만 사용하여 흰색 차선을 추출(`H: 0~180, S: 0~50, V: 180~255`)합니다. 실내 조명 반사나 어두운 노면 환경에서는 명도(V) 기준이 무너져 흰색 차선을 검출하지 못하는 현상이 발생합니다.
* **고도화 노드 우수성**: 흰색 차선에는 명도(Lightness) 채널이 독립된 **HLS 공간**, 노란색 차선에는 **HSV 공간**을 이중 채택하고 v1.7.0 튜닝(HLS L_low=120)을 통해 어두운 실내 트랙에서도 높은 이진 마스크 개구율을 확보합니다.

---

## 🛠️ 4. 단계별 개선 방향 및 실행 계획 (Action Plan)

### [1단계] 실행 노드 교체 및 제어 루프 복원 (즉시 조치)
현재 실행 중인 구형 `opencv_line_detect_node`를 종료하고, 고도화 비전 노드와 PID 제어 노드를 구동합니다.

```bash
# 1. 구형 노드 종료
ros2 node kill /opencv_line_detect_node

# 2. 고도화 차선 인지 노드 구동
ros2 run line_detection computer_vision_node --ros-args -p image_topic:=/image_raw

# 3. PID 차량 구동 제어 노드 구동
ros2 run vehicle_control drive_node
```

### [2단계] YOLO 추론 하드웨어 가속화 (CPU 637% -> GPU/TensorRT)
CPU 추론으로 인한 7.75 Hz 병목 현상 및 CPU 과부하를 해소하기 위해 CUDA/TensorRT 가속을 활성화합니다.

```bash
ros2 launch line_detection yolo_line_detection.launch.py \
  device:=cuda:0 \
  model_path:=/home/juwnoo/Downloads/road_best.engine
```

### [3단계] 토픽 QoS 및 통신 대역폭 최적화
모든 비전 수신 노드의 QoS Profile을 `BEST_EFFORT` 및 `KEEP_LAST (depth=1)`로 통일하여 대역폭 낭비와 과거 프레임 지연을 차단합니다.

---

## 📌 5. 결론 및 향후 기대 효과

본 개선안을 적용할 경우 시스템 정량적 지표는 다음과 같이 극적으로 개선됩니다.

| 측정 지표 | 현재 상태 | 개선 후 목표치 | 개선 효과 |
| :--- | :--- | :--- | :--- |
| **차량 제어 여부 (`/cmd_vel`)** | **0.0 Hz (불능)** | **20.0 Hz (정상)** | **자율주행 주포 제어 복원** |
| **YOLO 추론 지연시간** | **111.8 ms** | **< 15.0 ms** | **약 7.4배 속도 향상** |
| **YOLO CPU 점유율** | **637 %** | **< 40 %** | **시스템 자원 90% 이상 절감** |
| **비전 모듈 평가 점수** | **45 / 100 점** | **95 / 100 점** | **정밀 곡선 추종 및 조향 오차 신호 확보** |

---

## 🧾 6. 개선 적용 이력

### 2026-07-24 — 제어 루프 즉시 복원

보고서의 1단계 조치를 적용하여, 기존 `opencv_line_detect_node`를 종료하고 아래 노드 구성을 실행했다.

```text
/video_publisher_node
  └─ /image_raw
       └─ /computer_vision_node
            ├─ /lane_detection/steering_error
            │    └─ /drive_node
            │         └─ /cmd_vel
            ├─ /lane_detection/debug_image
            └─ /lane_detection/binary_mask
                 └─ /debug_node
```

- `/computer_vision_node`는 `/image_raw`를 구독하고, 차선 오차·BEV 디버그 영상·이진 마스크를 발행하도록 실행했다.
- `/drive_node`는 `/lane_detection/steering_error`를 구독하고 `/cmd_vel`을 발행하도록 실행했다.
- `/debug_node`의 입력을 `/lane_detection/debug_image`, `/lane_detection/binary_mask`, `/lane_detection/steering_error`, `/cmd_vel`로 연결해 HUD와 `/debug/composite_image` 출력을 유지했다.
- 실행 검증에서 `/cmd_vel` 발행 주기는 약 20 Hz로 확인했다.

### 2026-07-24 — 추종 경로 플래닝 및 HUD 오버레이

- `line_planner` 패키지를 추가하고 `/lane_detection/binary_mask`에서 차선 중심점을 샘플링해 `/planned_path`(`nav_msgs/Path`)를 발행하도록 구현했다.
- 경로 좌표는 `bev_normalized` 프레임에서 0.0~1.0으로 정규화해 디버그 화면의 BEV 좌표계와 직접 대응시켰다.
- `/debug_node`가 `/planned_path`를 구독하도록 확장하고, 세 번째 BEV 패널에 청록색 `Tracking Path` 폴리라인을 오버레이했다.
- 실행 검증에서 `/line_planner_node`의 `/planned_path` 발행과 `/debug_node`의 구독 연결을 확인했다.

### 2026-07-24 — 반사 노이즈 저감 및 YOLO-OpenCV 융합

- 차선 마스크에 밝기 에지 결합, 세로형 연결요소 필터, 강건 다항식 피팅 및 프레임 간 곡선 평활화를 추가해 반사광 노이즈의 영향을 줄였다.
- 흰색 차선 HLS 채도 상한을 180에서 100으로 조정해 고채도 반사 픽셀의 유입을 줄였다.
- `/line_planner_node`가 `/yolo/detections`를 추가로 구독하도록 확장했다. `lane1`, `lane2`의 신뢰도 있는 YOLO 박스를 동일한 BEV 투영으로 변환해 OpenCV 마스크를 제한한다.
- YOLO 결과가 0.35초 이상 오래되었거나 OpenCV 마스크와 충분히 겹치지 않으면 OpenCV 단독 경로로 폴백한다.
- 융합 마스크는 `/planner/fused_mask`로 발행하고, `/debug_node`가 이를 HUD의 마스크 패널에 표시하도록 연결했다.

### 실행 제약 및 후속 조치

호스트 디스크 여유 공간이 0B여서 워크스페이스 재빌드는 수행하지 못했다. 따라서 이번 전환은 최신 소스 모듈을 직접 실행하는 방식으로 적용했다. 디스크 공간 확보 후 `colcon build --symlink-install`을 수행하여 `computer_vision_node`, `drive_node`, `debug_node`를 설치 산출물과 런치 구성에 반영해야 한다.

GPU/TensorRT 전환과 성능 수치 재측정은 CUDA/TensorRT 환경 및 엔진 모델 준비 후 별도 검증 단계에서 진행한다.
