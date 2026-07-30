# 협업 및 Pull Request 가이드

> **2026 운영 원칙:** 상위 통합 Issue 하나를 한 사람이 모두 맡지 않습니다. 같은 Issue에 여러 담당자가 OpenCV·YOLO·LiDAR·학습·퓨전 같은 구체적인 슬롯으로 들어가 각자 브랜치와 PR을 제출합니다. 개인 PR에는 `Refs #이슈번호`를 사용하고, 필요한 여러 PR이 합쳐져 통합 완료 조건을 충족했을 때만 Issue를 닫습니다. 최신 역할과 현황은 [메인 README](./README.md)를 우선합니다.
>
> 

이 문서는 KHU Dolsoe Autonomous 팀의 GitHub 작업 규칙입니다.
## 0. Git과 GitHub가 처음인 팀원 안내

### Git이란?

Git은 코드와 문서의 변경 이력을 기록하는 프로그램입니다.

예를 들어 코드를 수정했다가 문제가 생겼을 때 이전 상태를 확인하거나, 누가 어떤 파일을 변경했는지 추적할 수 있습니다. Git은 기본적으로 자신의 컴퓨터에서 동작합니다.

```text
내 컴퓨터에서 파일 수정
→ Git이 변경사항 확인
→ commit으로 변경 이력 저장
```

### GitHub란?

GitHub는 Git으로 관리하는 저장소를 인터넷에서 팀원들과 공유하는 서비스입니다.

GitHub에서는 다음 작업을 할 수 있습니다.

- 팀 코드를 온라인에 보관
- 팀원별 작업 브랜치 관리
- 해야 할 일을 Issue로 등록
- Pull Request로 코드 제출
- 코드 리뷰와 의견 작성
- 프로젝트 진행 상태 관리
- Release로 검증된 버전 배포

정리하면 다음과 같습니다.

```text
Git = 내 컴퓨터에서 변경 이력을 관리하는 프로그램
GitHub = Git 저장소를 팀원들과 공유하는 온라인 서비스
```

### 로컬과 원격 저장소

Git을 사용할 때는 같은 프로젝트가 두 곳에 존재합니다.

#### 로컬 저장소

자신의 노트북에 있는 프로젝트 폴더입니다.

```text
WSL 또는 Ubuntu에 있는 KHU_Dolsoe_Autonomous 폴더
```

이곳에서 코드를 작성하고 실행합니다.

#### 원격 저장소

GitHub에 올라가 있는 온라인 저장소입니다.

```text
https://github.com/sheepmeat/KHU_Dolsoe_Autonomous
```

팀원이 로컬에서 작업한 내용은 자동으로 GitHub에 올라가지 않습니다.

```text
파일 수정
→ commit
→ push
```

과정을 거쳐야 GitHub에 올라갑니다.

### Commit과 Push의 차이

`commit`은 현재 변경사항을 자신의 컴퓨터에 기록하는 것입니다.

```bash
git commit -m "feat: 신호등 노드 추가"
```

이 명령을 실행해도 아직 GitHub에는 올라가지 않습니다.

`push`는 로컬에 저장된 commit을 GitHub 브랜치로 업로드하는 것입니다.

```bash
git push
```

정리하면 다음과 같습니다.

```text
commit = 내 컴퓨터에 변경 이력 저장
push = 저장한 변경 이력을 GitHub에 업로드
```

### Pull과 Pull Request의 차이

두 용어는 이름이 비슷하지만 전혀 다른 기능입니다.

`git pull`은 GitHub에 있는 최신 코드를 자신의 컴퓨터로 가져오는 명령입니다.

```bash
git pull origin main
```

Pull Request는 자신의 작업 브랜치를 `main`에 합쳐달라고 요청하는 GitHub 제출서입니다.

```text
git pull = 최신 코드 다운로드
Pull Request = 내 코드를 main에 합쳐달라는 요청
```

### Issue란?

Issue는 해야 할 일을 기록하는 업무 카드입니다.

예시:

```text
#23 신호등 YOLO 추론 노드 제작
#24 차선 마스크 후처리
#25 LiDAR 라바콘 중심점 검출
```

Issue에는 다음 내용을 기록합니다.

- 무엇을 만들어야 하는지
- 담당자가 누구인지
- 필요한 입력 데이터
- 예상되는 출력
- 완료 조건
- 필요한 장비
- 현재 막힌 문제

코드 작업을 시작하기 전에 자신이 담당하는 Issue가 있어야 합니다.

### Branch란?

Branch는 다른 사람의 작업과 분리된 개인 작업 공간입니다.

`main`은 팀의 기준 코드이므로 실험 중인 코드를 직접 올리면 다른 팀원의 작업이 망가질 수 있습니다. 따라서 Issue마다 별도 브랜치를 만듭니다.

```text
main
 ├─ feat/23-traffic-light-node
 ├─ feat/24-lane-mask
 └─ fix/25-lidar-frame-id
```

각 팀원은 자신의 브랜치에서 작업하고, 검증된 결과만 Pull Request를 통해 `main`에 반영합니다.

### Pull Request란?

Pull Request, 줄여서 PR은 자신의 작업 결과를 팀에 제출하는 기능입니다.

PR에서는 다음 내용을 확인할 수 있습니다.

- 변경된 파일
- 추가되거나 삭제된 코드
- 작성한 commit
- 실행 방법
- 결과 이미지와 로그
- Reviewer의 질문과 수정 요청
- 테스트 통과 여부

PR은 단순 파일 업로드가 아니라 다음 요청을 의미합니다.

```text
제가 이 브랜치에서 이 작업을 했습니다.
내용을 검토하고 문제가 없다면 main에 합쳐주세요.
```

작업이 미완성이라도 `Draft Pull Request`를 만들 수 있습니다. Draft PR은 현재 진행 상황을 공유하고 조언을 받기 위한 PR입니다.

### Merge란?

Merge는 Pull Request의 변경사항을 `main`에 실제로 합치는 작업입니다.

```text
개인 브랜치
→ Pull Request
→ 리뷰
→ 승인
→ Merge
→ main 반영
```

Merge 전에는 다른 팀원이 코드를 검토하고 실행 결과를 확인합니다.

### 이 프로젝트의 기본 작업 예시

신호등 노드 Issue가 `#23`이라고 가정합니다.

```text
1. Issue #23의 담당자가 된다.
2. 최신 main을 받는다.
3. feat/23-traffic-light-node 브랜치를 만든다.
4. 신호등 노드를 개발한다.
5. 변경사항을 commit한다.
6. 개인 브랜치를 GitHub에 push한다.
7. Draft Pull Request를 만든다.
8. 실행 결과와 영상을 PR에 첨부한다.
9. Reviewer의 의견을 반영한다.
10. 승인을 받은 후 main에 Merge한다.
11. Issue #23을 완료 처리한다.
```

### Git이 관리하는 네 단계

파일 변경은 다음 네 단계를 거칩니다.

```text
작업 폴더
→ Staging Area
→ 로컬 Commit
→ GitHub
```

#### 1. 작업 폴더

파일을 직접 작성하고 수정하는 상태입니다.

#### 2. Staging Area

다음 commit에 포함할 파일을 선택한 상태입니다.

```bash
git add 파일명
```

#### 3. 로컬 Commit

선택한 변경사항을 자신의 컴퓨터에 기록한 상태입니다.

```bash
git commit -m "변경 내용"
```

#### 4. GitHub

로컬 commit을 온라인 브랜치에 업로드한 상태입니다.

```bash
git push
```

### 자주 보게 되는 이름

- `main`: 팀의 기준 브랜치
- `origin`: GitHub 원격 저장소를 가리키는 기본 이름
- `HEAD`: 현재 자신이 위치한 commit 또는 브랜치
- `staged`: 다음 commit에 들어가도록 선택된 변경사항
- `modified`: 수정했지만 아직 commit하지 않은 파일
- `untracked`: Git이 아직 관리하지 않는 새 파일
- `Assignee`: Issue의 담당자
- `Reviewer`: Pull Request를 검토하는 사람
- `base`: 변경사항을 받을 브랜치, 보통 `main`
- `compare`: 자신이 작업한 브랜치


## 1. 핵심 규칙

- `main` 브랜치에 직접 push하지 않습니다.
- 작업을 시작하기 전에 GitHub Issue를 확인합니다.
- Issue 하나마다 새로운 작업 브랜치를 만듭니다.
- 작업 브랜치를 push한 후 Pull Request를 생성합니다.
- 한 Pull Request에는 한 가지 작업만 포함합니다.
- 리뷰와 검증을 통과한 후에만 `main`에 병합합니다.

전체 흐름:

```text
Issue 확인
→ 최신 main 받기
→ 작업 브랜치 생성
→ 코드 작성
→ commit
→ 작업 브랜치 push
→ Draft Pull Request
→ 리뷰 및 수정
→ merge
```

## 2. Git 용어

- `clone`: GitHub 저장소를 내 컴퓨터로 복사
- `branch`: 다른 사람의 코드에 영향을 주지 않는 작업 공간
- `commit`: 변경사항을 하나의 기록으로 저장
- `push`: 로컬 commit을 GitHub 브랜치에 업로드
- `pull`: GitHub의 최신 변경사항을 내 컴퓨터로 받기
- `Pull Request`: 작업 브랜치를 `main`에 합쳐달라는 요청
- `merge`: 승인된 Pull Request를 `main`에 합치기

`git pull`과 Pull Request는 서로 다른 기능입니다.

## 3. 최초 설정

Git 사용자 정보를 설정합니다.

```bash
git config --global user.name "본인 GitHub 이름"
git config --global user.email "본인 GitHub 이메일"
```

저장소를 복제합니다.

```bash
git clone https://github.com/sheepmeat/KHU_Dolsoe_Autonomous.git
cd KHU_Dolsoe_Autonomous
```

상태를 확인합니다.

```bash
git status
git remote -v
git branch
```

## 4. 작업 시작

먼저 GitHub에서 담당 Issue 번호를 확인합니다.

최신 `main`을 받습니다.

```bash
git switch main
git pull origin main
```

Issue 번호에 맞춰 새로운 브랜치를 만듭니다.

```bash
git switch -c feat/23-traffic-light-node
```

브랜치 이름 규칙:

```text
feat/이슈번호-기능
fix/이슈번호-버그
data/이슈번호-데이터
docs/이슈번호-문서
test/이슈번호-시험
```

예시:

```text
feat/23-traffic-light-node
fix/41-lidar-frame-id
data/52-cone-dataset
docs/60-jetson-setup
```

브랜치 이름에는 한글과 공백을 사용하지 않습니다.

## 5. 변경사항 저장

변경된 파일을 확인합니다.

```bash
git status
git diff
```

필요한 파일만 선택합니다.

```bash
git add 변경한파일
```

예시:

```bash
git add src/traffic_light/traffic_light_node.py
git add config/traffic_light.yaml
```

Commit 전에 선택된 내용을 확인합니다.

```bash
git diff --staged
```

Commit을 생성합니다.

```bash
git commit -m "feat: 신호등 추론 노드 추가"
```

추천 commit 종류:

```text
feat: 새로운 기능
fix: 버그 수정
data: 데이터와 라벨 변경
docs: 문서 변경
test: 테스트 추가
refactor: 동작 변경 없는 코드 정리
chore: 설정 또는 기타 작업
```

## 6. 작업 브랜치 Push

첫 push:

```bash
git push -u origin feat/23-traffic-light-node
```

이후 같은 브랜치에서 추가로 push할 때:

```bash
git push
```

다음 명령은 사용하지 않습니다.

```bash
git push origin main
git push --force
```

## 7. Pull Request 생성

브랜치를 push한 후 GitHub 저장소로 이동합니다.

1. `Compare & pull request`를 클릭합니다.
2. `base`가 `main`인지 확인합니다.
3. `compare`가 자신의 작업 브랜치인지 확인합니다.
4. 관련 Issue 번호를 작성합니다.
5. 변경 내용과 실행 방법을 작성합니다.
6. 결과 이미지·영상·로그를 첨부합니다.
7. 미완성 작업은 `Draft Pull Request`로 생성합니다.
8. 완성된 작업은 `Ready for review`로 전환합니다.
9. 같은 파트 팀원 또는 통합 담당자를 Reviewer로 지정합니다.

미완성 상태여도 정해진 제출일까지 브랜치를 push하고 Draft PR을 생성합니다.

## 8. Issue 연결

PR 병합과 동시에 Issue도 완료되는 작업:

```text
Closes #23
```

젯슨·차량·LiDAR·트랙 시험이 남아 있는 작업:

```text
Refs #23
```

`Refs`를 사용하면 PR이 병합돼도 Issue는 자동으로 닫히지 않습니다. 장비 검증이 끝난 후 Issue를 닫습니다.

## 9. 리뷰 의견 반영

Reviewer가 수정을 요청해도 새로운 PR을 만들지 않습니다.

같은 브랜치에서 수정합니다.

```bash
git add 수정한파일
git commit -m "fix: 리뷰 의견 반영"
git push
```

기존 PR이 자동으로 갱신됩니다.

## 10. Merge 후 정리

PR이 병합된 후 로컬 저장소를 정리합니다.

```bash
git switch main
git pull origin main
git branch -d feat/23-traffic-light-node
```

다음 작업은 최신 `main`에서 새로운 브랜치를 만들어 시작합니다.

## 11. 제출 완료 기준

Pull Request에는 다음 항목이 있어야 합니다.

- 관련 Issue
- 변경한 코드
- 설치 및 실행 명령
- 입력 데이터·영상·rosbag 버전
- 출력 결과
- 정확도, FPS 또는 지연시간
- 결과 이미지·영상·로그
- 필요한 장비
- 알려진 문제
- 다음 행동

`.pt` 모델 제출 시 다음 정보도 작성합니다.

- 데이터셋 버전
- train/validation/test 분할
- 학습 명령
- 학습 설정
- mAP, precision, recall
- 사용한 코드 commit
- 모델 checksum
- 실패 사례

## 12. 올리면 안 되는 파일

다음 파일은 일반 Git commit에 포함하지 않습니다.

- 원본 데이터셋
- 대용량 영상
- rosbag
- `.pt` 모델
- `build/`, `install/`, `log/`
- YOLO `runs/`
- Python 가상환경
- 비밀번호, API 키, 토큰
- `.env` 파일

파일을 추가하기 전에 반드시 확인합니다.

```bash
git status
git diff --staged
```

## 13. 문제 발생 시

다음 문제가 발생하면 임의로 덮어쓰거나 강제 push하지 않습니다.

- Merge conflict
- push rejected
- 실수로 큰 파일 commit
- 다른 사람 파일 삭제
- 브랜치를 잘못 생성
- `main`에서 작업
- 비밀번호나 토큰 commit

담당 Issue에 다음 내용을 작성하고 도움을 요청합니다.

```text
실행한 명령:
발생한 오류:
현재 브랜치:
git status 결과:
해결을 위해 시도한 내용:
```

## 14. 팀 운영 규칙

- 한 사람은 동시에 하나의 Issue만 `In Progress`로 진행합니다.
- 작업 시작 시 Issue를 `In Progress`로 이동합니다.
- Draft PR을 생성해도 개발 중이면 `In Progress`를 유지합니다.
- 리뷰 준비가 완료되면 `Review`로 이동합니다.
- 오프라인 검증을 통과하면 `Integration Queue`로 이동합니다.
- 실제 장비 시험 중에는 `Hardware Test`로 이동합니다.
- 진행이 불가능하면 `Blocked`로 이동하고 이유를 작성합니다.
- PR 병합과 요구된 시험이 모두 끝난 경우에만 `Done`으로 이동합니다.
