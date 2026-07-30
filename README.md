# KHU Dolsoe Autonomous

국민대학교 자율주행 경진대회를 준비하는 경희대학교 자율주행팀 저장소입니다.

## 팀원 작업 시작 및 제출 방법

### 1. 담당 작업 확인

[이번 주 작업 보드](https://github.com/users/sheepmeat/projects/1/views/3)에서 본인 이름이 지정된 Issue를 확인합니다.

- 작업 시작 전: `Ready`
- 작업 시작: `In Progress`
- Push 및 PR 제출: `Review`
- Jetson·LiDAR·차량 시험 대기: `Integration Queue`
- 실제 장비 시험 중: `Hardware Test`

### 2. 저장소 다운로드

GitHub Desktop을 설치하고 로그인합니다.

1. `File → Clone repository`
2. `sheepmeat/KHU_Dolsoe_Autonomous` 선택
3. `Clone`

터미널을 사용하는 경우:

```bash
git clone https://github.com/sheepmeat/KHU_Dolsoe_Autonomous.git
cd KHU_Dolsoe_Autonomous
```

### 3. 작업 브랜치 생성

GitHub Desktop:

`Current branch → New branch`

브랜치 이름:

`work/이슈번호-작업명`

예시:

- `work/12-traffic-yolo`
- `work/15-lane-mask`
- `work/21-lidar-cone`

터미널:

```bash
git switch main
git pull origin main
git switch -c work/이슈번호-작업명
```

### 4. 파일 배치

이번 토요일 제출은 저장소 루트의 본인 폴더에 올립니다.

```text
본인GitHub아이디/
├── README.md
├── 소스코드 또는 ROS workspace
├── config/
└── 결과 파일 또는 링크
```

본인 폴더의 `README.md`에 다음 내용을 작성합니다.

- 현재까지 완료한 것
- 실행 환경
- 실행 명령어
- 입력과 출력
- 결과 이미지·영상·로그
- 아직 안 된 것
- 현재 막힌 문제
- 다음에 할 일

데이터셋, `.pt`, rosbag, 대용량 영상은 GitHub에 직접 올리지 않고 외부 저장소 링크를 README에 작성합니다.

### 5. Push

GitHub Desktop:

1. 변경 파일 확인
2. Summary 작성
3. `Commit to work/...`
4. `Push origin`
5. `Create Pull Request`

터미널:

```bash
git add 본인폴더
git commit -m "submit: #이슈번호 현재 작업 제출"
git push -u origin work/이슈번호-작업명
```

### 6. Pull Request 작성

PR 제목:

`[SUBMIT] #이슈번호 이름 - 작업명`

PR 본문의 `Refs #` 뒤에 담당 Issue 번호를 작성합니다.

예시:

```text
Refs #12
```

작업이 완성되지 않았어도 토요일까지 현재 코드와 결과를 Push하고 PR을 생성합니다.

Git과 GitHub에 대한 자세한 설명은 [협업 및 Pull Request 가이드](CONTRIBUTING.md)를 확인하세요.
## 주요 개발 항목

- 신호등 인식
- 차선 검출 및 마스킹
- 장애물 YOLO 학습 및 ROS 노드
- LiDAR 기반 라바콘 검출·회피
- 경로 퓨전 및 차량 제어
- 차량 부품 3D 모델링·프린팅
- Jetson 및 실차 통합

## 작업 현황

- [GitHub Project 보드](https://github.com/users/sheepmeat/projects/1)
- [Issues](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/issues)
- [Pull Requests](https://github.com/sheepmeat/KHU_Dolsoe_Autonomous/pulls)

## 기본 협업 규칙

1. 모든 작업은 GitHub Issue와 연결합니다.
2. `main` 브랜치에 직접 push하지 않습니다.
3. Issue별 개인 브랜치를 생성합니다.
4. 개인 브랜치를 push한 후 Pull Request를 만듭니다.
5. 실행 명령과 결과 자료를 Pull Request에 작성합니다.
6. 리뷰와 검증을 통과한 작업만 `main`에 병합합니다.
7. 데이터셋, rosbag, 모델 파일은 지정된 저장 위치를 사용합니다.

자세한 사용법은 [협업 및 Pull Request 가이드](CONTRIBUTING.md)를 확인하세요.
