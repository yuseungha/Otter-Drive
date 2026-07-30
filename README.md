# KHU Dolsoe Autonomous

국민대학교 자율주행 경진대회를 준비하는 경희대학교 자율주행팀 저장소입니다. **진행 확인은 GitHub Project, 코드·모델 설정 제출은 개인 브랜치와 Pull Request(PR)** 를 사용합니다.

> 처음 온 팀원은 아래 `지금 바로 할 일`만 순서대로 진행하세요.

## 핵심 업무 방식

- 상위 Issue 하나는 **한 사람의 할 일**이 아니라 여러 노드와 여러 PR이 합쳐지는 **통합 목표**입니다.
- 한 Issue에 여러 명이 Assignee로 들어갈 수 있고, 각자 서로 다른 담당 슬롯을 골라 병렬로 개발합니다.
- 같은 문제를 OpenCV·YOLO·LiDAR·공개 구현 등 여러 방식으로 동시에 풀어도 됩니다.
- 후보 구현과 PT 파일은 같은 입력에서 비교한 뒤 최종 주행 노드에 채택합니다.
- 개인 완료는 `내가 맡은 노드 또는 학습 결과가 실행되는 것`, 통합 완료는 `선택된 후보들이 실제 주행 흐름으로 연결되는 것`입니다.

영상 실패 사례 정리나 새 노트북 재현 시험을 별도 인원 업무로 배정하지 않습니다. 지금은 각 담당자가 **잘 동작하는 노드·PT·CAD 결과를 직접 만드는 것**이 우선입니다.

## 지금 바로 할 일

1. [이번 주 작업 보드](https://github.com/users/sheepmeat/projects/1/views/3)를 엽니다.
2. 아래 5개 통합 Issue 중 하나를 열고 `슬롯 B 맡겠습니다`처럼 댓글로 구체적인 조각을 선택합니다.
3. 본인을 해당 Issue의 `Assignee`에 추가합니다. 같은 Issue에 여러 명이 들어가도 됩니다.
4. 작업 시작 시 Project 상태를 `In Progress`로 바꿉니다.
5. 개인 브랜치에 현재 결과를 Push하고 `Refs #이슈번호`로 PR을 연결합니다.
6. 막혔으면 Issue의 `Blocker` 또는 댓글에 막힌 이유와 필요한 도움을 적고 상태를 `Blocked`로 바꿉니다.

완성 전이어도 현재 코드, 실행 방법, 학습 로그, 결과 화면을 올리면 됩니다. 장비가 없으면 녹화 영상·rosbag·Mock 입력으로 먼저 개발합니다.

## 5개 통합 업무

| 통합 업무 | 여러 사람이 나눠 만드는 것 | Issue |
|---|---|---|
| 차선·YOLO·경로 | OpenCV 차선 / YOLO 차선 / 경로 계산 / ROS 연결 | [#8](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/8) |
| 장애물·신호등·LiDAR | 카메라 YOLO / LiDAR 회피 / 공개 구현 적용 / 신호등 / 회피 퓨전 | [#9](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/9) |
| Jetson·차량 | 실행환경 / 수동주행 기준선 / 차선 실차 / 라바콘 실차 / 시험 운영 | [#10](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/10) |
| CAD·프린팅 | CAD / 부품별 외부 출력·수령 / 장착 검증 / 외관 하우징 | [#11](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/11) |
| YOLO 학습·PT 검증 | 데이터셋별 병렬 학습 / 여러 PT 생성 / 공통 벤치마크 / 모델 교체 | [#14](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/14) |

## 역할을 나누는 예시

아래 역할은 직급이 아니라 **각자 제출할 결과물의 경계**입니다. 한 사람이 여러 역할을 잡기보다 먼저 한 조각을 실행 가능한 상태로 제출합니다.

| 분야 | 담당 슬롯 | 제출할 실제 결과 |
|---|---|---|
| 차선 | OpenCV·마스킹 | 중심선 또는 waypoint를 출력하는 노드 |
| 차선 | YOLO 차선 | YOLO 검출에서 경로 후보를 출력하는 노드 |
| 차선 | 경로 퓨전 | 후보 결과를 차량용 최종 경로로 변환 |
| 모델 | 데이터셋 후보 A/B | 서로 다른 데이터로 학습한 후보 PT |
| 모델 | 증강·설정 후보 | imgsz·epoch·증강 등을 바꾼 후보 PT |
| 모델 | 공통 벤치마크 | 같은 영상에서 모든 PT의 지표·FPS 비교표 |
| 모델 | YOLO 노드 적용 | `model_path` 설정만 바꿔 PT 교체 실행 |
| 장애물 | 카메라 YOLO | 장애물·라바콘 클래스와 좌표 출력 |
| 장애물 | LiDAR | 라바콘 좌표와 회피 경로 출력 |
| 장애물 | 공개 구현 적용 | 퍼블릭 GitHub 코드·데이터를 ROS 환경에 적용 |
| 장애물 | 회피 퓨전 | YOLO·LiDAR 후보 출력을 주행 명령에 연결 |
| Jetson | 실행환경 | 공통 launch와 실행 순서 |
| Jetson | 수동주행 기준선 | 모터·조향·비상정지 절차 |
| Jetson | 차선·라바콘 실차 | 각 주행 노드의 저속 차량 시험 |
| CAD | CAD 원본 | 치수·STL·STEP·버전 관리 |
| CAD | 외부 출력·수령 | 부품별 출력 상태·수령·사진 |
| CAD | 외관 하우징 | 아직 미설계인 차량 덮개 설계 |

## 현재 확인된 하드웨어 상태

### Jetson·차량

- 사용 가능한 Jetson은 1대이며 다른 1대는 메인보드 교체 전까지 사용 불가입니다.
- `@yuseungha`가 최근 실제 차량 수동주행을 수행했습니다.
- 라바콘 회피 실차와 실제 도로·차선 자율주행은 아직 미시험입니다.
- 국민대 실제 트랙은 아직 방문·시험하지 못했습니다.
- 경희대 도로·주차 현수막은 1차 저속 시험에 활용합니다.

### 3D 프린팅

- CAD 모델링 주담당은 `@ireeom-811`입니다.
- 최종적으로 부품 2세트가 필요하며 현재는 각 1개 시제품을 먼저 검증하는 단계입니다.
- 메인판 지지대와 메인 바스킷은 오늘 수령 예정입니다.
- 메인 바스킷 뚜껑 겸 Jetson 받침대는 아직 미인쇄입니다.
- LiDAR 커버·배터리 커버·카메라 고정대는 1차 출력 진행 대상으로 관리합니다.
- 차량 전체를 덮는 외관 하우징은 아직 미설계입니다.
- 부품별 최신 상태와 담당 슬롯은 [#11](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/11)에서 확인합니다.

## Project 상태 뜻

- `Backlog`: 해야 하지만 담당 슬롯이 아직 정해지지 않은 일
- `Ready`: 담당 슬롯과 제출물이 정해져 시작 가능한 일
- `In Progress`: 개인 브랜치에서 노드·학습·설계를 실제 진행 중
- `Review`: PR과 실행 결과를 확인하는 중
- `Integration Queue`: 개인 결과가 동작해 다른 노드 또는 장비 연결을 기다리는 중
- `Hardware Test`: Jetson·LiDAR·차량·출력물로 실제 검증 중
- `Blocked`: 자원·정보·오류 때문에 진행 불가. 이유를 반드시 기록
- `Done`: 필요한 개인 PR들이 병합되고 통합 완료 조건까지 충족

## 처음 한 번: 저장소 받기

### GitHub Desktop

1. GitHub Desktop을 설치하고 로그인합니다.
2. `File → Clone repository`에서 `sheepmeat/KHU_Dolsoe_Autonomous`를 선택합니다.
3. 저장할 위치를 고르고 `Clone`을 누릅니다.

### 터미널 / WSL / Ubuntu

```bash
git clone https://github.com/sheepmeat/KHU_Dolsoe_Autonomous.git
cd KHU_Dolsoe_Autonomous
```

WSL을 쓰면 Linux 홈 폴더 아래(예: `~/KHU_Dolsoe_Autonomous`)에 Clone하는 것을 권장합니다.

## 내 워크스페이스 넣는 위치

저장소 최상위에 **본인 GitHub 아이디 폴더 하나**를 만들고 그 안에 작업을 넣습니다.

```text
KHU_Dolsoe_Autonomous/
├── 본인GitHub아이디/
│   ├── README.md
│   ├── ros2_ws/ 또는 소스코드/
│   ├── launch/·config/·학습 스크립트/
│   └── 결과 이미지 또는 외부 링크
├── junwoo/
├── seungha/
└── README.md
```

ROS2 workspace 전체를 넣어도 되지만 `build/`, `install/`, `log/`, `.venv/`, 캐시 파일은 올리지 않습니다. 데이터셋, rosbag, `.pt`, 대용량 영상은 공유 Drive에 올리고 내 폴더의 `README.md`에 링크만 남깁니다.

내 폴더의 `README.md`에는 최소한 다음을 적습니다.

- 내가 맡은 Issue 번호와 담당 슬롯
- 현재까지 완료한 것
- Ubuntu·ROS·Python·YOLO 등 실행 환경
- 설치·학습·실행 명령어
- 입력과 출력 토픽 또는 파일
- 결과 이미지·영상·로그·PT 링크
- 아직 안 된 것과 현재 Blocker

## 매 작업 시작: 최신 main에서 브랜치 만들기

브랜치는 담당 슬롯마다 새로 만듭니다. 이름은 `work/이슈번호-짧은작업명`으로 통일합니다.

### GitHub Desktop

1. `Current branch`에서 `main` 선택
2. `Fetch origin` 후 `Pull origin`이 보이면 실행
3. `Current branch → New branch`
4. 예: `work/14-yolo-train-a` 또는 `work/9-lidar-cone`

### 터미널

```bash
git switch main
git pull origin main
git switch -c work/14-yolo-train-a
```

## 현재 작업 Push하기

### GitHub Desktop

1. 변경 파일에서 대용량 파일과 비밀키가 없는지 확인합니다.
2. `Summary`에 작업 내용을 적습니다.
3. `Commit to work/...` → `Push origin` → `Create Pull Request` 순서로 누릅니다.

### 터미널

```bash
git status
git add 본인GitHub아이디/
git commit -m "feat: #14 데이터셋 A 학습 후보 제출"
git push -u origin work/14-yolo-train-a
```

`git add .`를 쓰기 전에는 반드시 `git status`로 대용량 파일과 비밀키가 포함되지 않았는지 확인하세요.

## Pull Request 만들기

1. Push 후 GitHub의 `Compare & pull request`를 누릅니다.
2. `base: main`, `compare: 내 작업 브랜치`인지 확인합니다.
3. 제목은 `[SUBMIT] #이슈번호 이름 - 담당 슬롯`으로 적습니다.
4. PR 양식에 실행 방법, 결과, 아직 안 된 것, 장비 시험 필요 여부를 적습니다.
5. 본문에 `Refs #14`처럼 담당 Issue를 연결합니다.
6. Project 상태가 `Review`로 이동했는지 확인합니다.

하나의 통합 Issue에 여러 PR이 연결되므로 개인 PR에서는 `Closes #번호`가 아니라 `Refs #번호`를 사용합니다. 최종 통합이 끝났을 때만 Issue를 닫습니다.

## 같은 브랜치에 결과 추가하기

기존 PR이 있으면 새 PR을 만들지 않고 같은 브랜치에서 다시 Commit하고 Push합니다.

```bash
git status
git add 본인GitHub아이디/
git commit -m "fix: 검출 노드 수정"
git push
```

## 꼭 지킬 최소 규칙

- `main`에 직접 Push하지 않습니다. 개인 브랜치 → PR 순서로 제출합니다.
- 다른 사람이 같은 Issue를 맡아도 각자 담당 슬롯과 브랜치를 분리합니다.
- 코드·학습·CAD 결과와 함께 실행 명령과 결과 한 개 이상을 남깁니다.
- 데이터셋, rosbag, `.pt`, CAD 대용량 파일은 공유 Drive 링크로 남깁니다.
- 공개 데이터·코드는 출처와 라이선스, 우리가 수정한 부분을 기록합니다.
- 비밀번호, 토큰, 개인키, `.env`는 절대 올리지 않습니다.
- 장비 작업은 개인 입력 검증 후 `Integration Queue`로 보내 여러 PR을 묶어 시험합니다.

## 바로가기

- [이번 주 작업 보드](https://github.com/users/sheepmeat/projects/1/views/3)
- [작업 요청 만들기](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/new/choose)
- [Issues](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues)
- [Pull Requests](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/pulls)
- [자세한 Git·PR 설명](./CONTRIBUTING.md)
