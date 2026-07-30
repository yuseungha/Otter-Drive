# KHU Dolsoe Autonomous

국민대학교 자율주행 경진대회를 준비하는 경희대학교 자율주행팀 저장소입니다. **작업 확인은 Project, 코드 제출은 개인 브랜치와 Pull Request(PR)** 를 사용합니다.

> 처음 온 팀원은 아래 **지금 바로 할 일**부터 순서대로 진행하세요.

## 지금 바로 할 일

1. [이번 주 작업 보드](https://github.com/users/sheepmeat/projects/1/views/3)를 엽니다.
2. 본인이 맡을 4개 통합 Issue 중 하나를 열고 댓글에 역할을 남긴 뒤 본인을 `Assignee`로 추가합니다.
3. 작업을 시작할 때 상태를 `In Progress`로 바꿉니다.
4. 아래 방법대로 개인 브랜치에 현재 결과를 Push하고 PR을 만듭니다.
5. 막혔으면 멈춰 있지 말고 Issue의 `Blocker` 또는 댓글에 **막힌 이유와 필요한 도움**을 적고 상태를 `Blocked`로 바꿉니다.

완성 전이어도 현재 코드, 실행 방법, 결과 로그를 올리면 됩니다. 장비가 없으면 녹화 영상·rosbag·Mock 입력으로 개인 개발을 먼저 진행합니다.

## 4개 통합 작업과 역할

| 통합 작업 | 주담당 | 지원 역할 예시 | Issue |
|---|---|---|---|
| 차선·YOLO·경로 | 통합 노드와 경로 출력 책임 | 영상·데이터 정리 / 새 환경 재현 시험 | [#8](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/8) |
| 장애물·신호등·LiDAR | 인식과 회피 통합 책임 | 데이터·모델 학습 / rosbag·Mock·LiDAR 시험 | [#9](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/9) |
| Jetson·차량 | Jetson 실행환경과 실차 운용 책임 | 설치 스크립트 / 시험 로그·체크리스트 | [#10](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/10) |
| CAD·프린팅 | 설계와 장착 치수 책임 | 외부 출력 / 후가공·차량 장착 확인 | [#11](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/11) |

한 분야에 주담당 1명과 지원 1~2명이 붙습니다. 역할이 비어 있으면 해당 Issue 댓글에 `데이터 역할 맡겠습니다`, `Jetson 로그 역할 맡겠습니다`처럼 먼저 선언하세요.

## Project 상태 뜻

- `Backlog`: 해야 하지만 이번에 바로 시작하지 않는 일
- `Ready`: 담당자와 목표가 정해져 시작 가능한 일
- `In Progress`: 개인 노트북 등에서 실제 작업 중
- `Review`: PR을 올려 코드·결과를 확인받는 중
- `Integration Queue`: 개인 검증이 끝나 Jetson·LiDAR·차량 통합을 기다리는 중
- `Hardware Test`: 장비를 확보해 대면 시험 중
- `Blocked`: 자원·정보·오류 때문에 진행 불가. 이유를 반드시 기록
- `Done`: 코드와 결과가 main에 반영되고 필요한 시험까지 끝남

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

WSL을 쓰면 가능하면 Linux 홈 폴더 아래(예: `~/KHU_Dolsoe_Autonomous`)에 Clone하세요.

## 내 워크스페이스 넣는 위치

저장소 최상위에 **본인 GitHub 아이디 폴더 하나**를 만들고 그 안에 작업을 넣습니다.

```text
KHU_Dolsoe_Autonomous/
├── 본인GitHub아이디/
│   ├── README.md
│   ├── ros2_ws/ 또는 소스코드/
│   ├── launch/·config/·models/ 등의 설정
│   └── 결과 이미지 또는 외부 링크
├── junwoo/
├── seungha/
└── README.md
```

ROS2 workspace 전체를 넣어도 되지만 `build/`, `install/`, `log/`, `.venv/`, 캐시 파일은 올리지 않습니다. 데이터셋, rosbag, `.pt`, 대용량 영상은 공유 Drive에 올리고 내 폴더의 `README.md`에 링크만 남깁니다.

내 폴더의 `README.md`에는 최소한 다음을 적습니다.

- 현재까지 완료한 것
- Ubuntu/ROS/Python 등 실행 환경
- 설치 및 실행 명령어
- 입력과 출력
- 결과 이미지·영상·로그 링크
- 아직 안 된 것과 현재 Blocker
- 다음에 할 일

## 매 작업 시작: 최신 main에서 브랜치 만들기

브랜치는 작업마다 새로 만듭니다. 이름은 `work/이슈번호-짧은작업명`으로 통일합니다.

### GitHub Desktop

1. `Current branch`에서 `main` 선택
2. `Fetch origin` 후 `Pull origin`이 보이면 실행
3. `Current branch → New branch`
4. 예: `work/8-lane-path` 입력 후 생성

### 터미널

```bash
git switch main
git pull origin main
git switch -c work/8-lane-path
```

## 현재 작업 Push하기

### GitHub Desktop

1. 왼쪽 변경 파일에서 올리면 안 되는 대용량 파일이 없는지 확인합니다.
2. 왼쪽 아래 `Summary`에 작업 내용을 적습니다.
3. `Commit to work/...`를 누릅니다.
4. 위쪽 `Push origin`을 누릅니다.
5. 처음 Push한 브랜치라면 `Create Pull Request`를 누릅니다.

### 터미널

```bash
git status
git add 본인GitHub아이디/
git commit -m "feat: #8 차선 경로 작업 중간 결과"
git push -u origin work/8-lane-path
```

`git add .`를 쓰기 전에는 반드시 `git status`로 대용량 파일과 비밀키가 포함되지 않았는지 확인하세요.

## Pull Request 만들기

1. Push 후 GitHub 저장소에 나타나는 `Compare & pull request`를 누릅니다.
2. `base: main`, `compare: 내 작업 브랜치`인지 확인합니다.
3. 제목을 `[SUBMIT] #이슈번호 이름 - 작업명`으로 적습니다.
4. 자동으로 보이는 PR 양식에 실행 방법, 결과, 아직 안 된 것, 장비 시험 필요 여부를 적습니다.
5. 본문에 `Refs #8`처럼 담당 Issue를 연결하고 `Create pull request`를 누릅니다.
6. Project 상태가 `Review`로 이동했는지 확인합니다.

완성 전 중간 제출이면 PR 제목이나 본문에 `WIP`라고 적으면 됩니다. 팀장 확인 없이 본인이 main에 병합하지 않습니다.

## 같은 브랜치에 두 번째·세 번째 결과 올리기

PR을 만든 뒤 수정이 생겨도 새 PR을 만들 필요가 없습니다. 같은 브랜치에서 다시 Commit하고 Push하면 기존 PR에 자동으로 추가됩니다.

```bash
git status
git add 본인GitHub아이디/
git commit -m "fix: 실행 오류 수정"
git push
```

해당 작업이 병합된 뒤 새 일을 시작할 때만 다시 `main`을 Pull하고 새 브랜치를 만듭니다.

## 꼭 지킬 최소 규칙

- `main`에 직접 Push하지 않습니다. 개인 브랜치 → PR 순서로 제출합니다.
- 작업 시작·중단·제출 때 Project 상태를 바꿉니다.
- 코드만 올리지 말고 실행 명령과 결과 한 개 이상을 남깁니다.
- 데이터셋, rosbag, `.pt`, 대용량 영상은 Git이 아니라 공유 Drive 링크로 남깁니다.
- 비밀번호, 토큰, 개인키, `.env`는 절대 올리지 않습니다.
- 장비가 필요한 작업은 개인 검증 후 `Integration Queue`로 보내 장비 시간을 묶어서 사용합니다.

## 바로가기

- [이번 주 작업 보드](https://github.com/users/sheepmeat/projects/1/views/3)
- [작업 요청 만들기](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues/new/choose)
- [Issues](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues)
- [Pull Requests](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/pulls)
- [자세한 Git·PR 설명](./CONTRIBUTING.md)
