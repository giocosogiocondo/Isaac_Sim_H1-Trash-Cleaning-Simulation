# Isaac Sim H1 Trash Cleanup (Teleop) 아직 수정할 부분 많고 수동 조작입니다 방향키로 조작하세용
(아마도 TSP 알고리즘을 적용하여 주행까지는 최적화하더라도 실제 청소는 직접 만들어야할거에요)

Unitree **H1**를 Isaac Sim에서 **키보드(방향키)로 조종**하면서, 랜덤으로 생성된 방/내부벽/쓰레기(큐브)를 탐색하고 **근접 유지 시 쓰레기를 제거**하는 데모입니다.

- 메인 실행 스크립트: `my_h1_policy_in_trash_env.py` :contentReference[oaicite:0]{index=0}
- 환경 생성: `env_gen.py` :contentReference[oaicite:1]{index=1}
- 빈 공간 샘플링 유틸: `trash_utils.py` :contentReference[oaicite:2]{index=2}

> 이 코드는 Isaac Sim 내장 Python 환경에서 실행하는 것을 전제로 합니다. (`isaacsim`, `omni.*`, `pxr.*` 모듈은 일반 Python에 없습니다.) :contentReference[oaicite:3]{index=3}

---

## 기능 요약

- 랜덤 방(외벽) 생성 + 내부 벽 랜덤 배치 (`env_gen.py`)   
- 랜덤 쓰레기(VisualCuboid) 스폰 (`env_gen.py`) :contentReference[oaicite:5]{index=5}
- Unitree H1(FlatTerrainPolicy) 스폰 (`my_h1_policy_in_trash_env.py`) :contentReference[oaicite:6]{index=6}
- 방향키 텔레옵 
  - **↑/↓**: 전진/후진
  - **←/→**: 제자리 회전
  - **Shift**: 부스트
  - **Space**: 즉시 정지(비상정지)
  - **R**: 월드 리셋
- 쓰레기 제거 로직: 로봇이 쓰레기 **반경 내에 일정 시간 머무르면 USD Prim 제거** :contentReference[oaicite:8]{index=8}

---

## 폴더 구조

아래처럼 **같은 폴더**에 3개 파일을 놓고 실행하세요.
trash_env/
├─ my_h1_policy_in_trash_env.py
├─ env_gen.py
└─ trash_utils.py


---

## 설치 / 요구사항

### 필수
- NVIDIA **Isaac Sim 4.5.0** :contentReference[oaicite:9]{index=9}
- Isaac Sim에 포함된 Python (일반 Python X) :contentReference[oaicite:10]{index=10}

### Python 패키지
Isaac Sim이 대부분의 의존성을 포함하지만, 이 프로젝트에서 코드상 명시적으로 사용하는 외부 패키지는 `numpy`입니다. 

---

## 실행 방법

> 핵심: **Isaac Sim 설치 폴더의 python 실행기**로 돌려야 합니다. :contentReference[oaicite:12]{index=12}

### Windows 예시
```bat
cd <ISAAC_SIM_ROOT>\
python.bat <PATH_TO>\trash_env\my_h1_policy_in_trash_env.py --num-robots 1 --seed 0
Linux 예시
cd <ISAAC_SIM_ROOT>/
./python.sh <PATH_TO>/trash_env/my_h1_policy_in_trash_env.py --num-robots 1 --seed 0
옵션
--num-robots: 스폰할 로봇 수 (기본 1) 
my_h1_policy_in_trash_env


--seed: 랜덤 시드 (생략하면 매번 랜덤) 
my_h1_policy_in_trash_env


조작 방법 (Teleop)
↑ / ↓ : 전진 / 후진

← / → : 좌회전 / 우회전 (yaw rate)

Shift : 속도 부스트

Space : 즉시 정지(키 입력 상태 초기화) 
my_h1_policy_in_trash_env


R : 월드 리셋 
my_h1_policy_in_trash_env


쓰레기 제거 로직
쓰레기 prim의 월드 좌표를 계산하고 
my_h1_policy_in_trash_env


로봇(/World/H1_0)과의 거리가 APPROACH_RADIUS 이하이면 dwell time을 누적 
my_h1_policy_in_trash_env


누적 시간이 WAIT_DURATION 이상이면 해당 쓰레기 prim을 stage.RemovePrim()으로 제거

기본값:

APPROACH_RADIUS = 1.0 (m), WAIT_DURATION = 3.0 (s) 
my_h1_policy_in_trash_env


주요 파라미터
my_h1_policy_in_trash_env.py
LIN_V, YAW_V, BOOST: 텔레옵 속도 상수 
my_h1_policy_in_trash_env


APPROACH_RADIUS, WAIT_DURATION: 쓰레기 제거 조건 
my_h1_policy_in_trash_env


env_gen.py
spawn_room_walls(): 방 크기 범위/벽 두께/높이 
env_gen


spawn_internal_walls(): 내부벽 개수/길이/두께 범위 
env_gen


spawn_trash(): 쓰레기 개수/크기/높이/벽과의 마진 
env_gen


참고: spawn_trash()에서 충돌은 기본적으로 꺼져 있습니다(주석 처리). 필요하면 충돌 enable을 켜세요. 
env_gen


트러블슈팅
ModuleNotFoundError: No module named 'isaacsim'
일반 Python이 아니라 Isaac Sim의 python.sh / python.bat로 실행해야 합니다. 
my_h1_policy_in_trash_env


assets_root_path is None
Isaac Sim assets 경로를 찾지 못한 경우입니다. 
my_h1_policy_in_trash_env


Failed to sample a free position
방이 너무 좁거나 내부벽/쓰레기/로봇 밀도가 높아 빈 자리를 못 찾는 경우입니다. room_size_range를 키우거나 개수를 줄이세요.
