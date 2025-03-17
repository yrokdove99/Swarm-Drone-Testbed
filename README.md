# Robomaster EP 기반 군집 로봇 테스트베드 구축 및 제어 알고리즘 구현

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/02173d7c-6904-47db-99b2-20c5cc657b2e" width="200"/></td>
    <td><img src="https://github.com/user-attachments/assets/e4e1c530-b803-40e1-a53f-263e01b3aca7" width="200"/></td>
    <td><img src="https://github.com/user-attachments/assets/9abeff15-d530-4a2f-93cf-0e21465c6bf9" width="200"/></td>
    <td rowspan="2"><img src="https://github.com/user-attachments/assets/9cccd273-e668-449f-af7c-282fc7f8fa97" width="400"/></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/f5d0a633-bb3e-47cb-9df2-1543d886eca9" width="200"/></td>
    <td><img src="https://github.com/user-attachments/assets/a7002302-eb93-49a2-8634-a670e2af97fd" width="200"/></td>
    <td><img src="https://github.com/user-attachments/assets/7f5f2b42-3aa2-4be8-8db7-f0e2f475e7c1" width="200"/></td>
    </tr>
</table>


## 설명
모션 캡쳐 시스템 Optitrack 장비 및 프로그램을 기반으로 구축하였습니다.

테스트베드 환경을 통해 6가지 기능을 구현하여 테스트해보았습니다.


## 설치 (Installation)

- HW
  - Optitrack 장비 사용

- SW
1. 개발환경
   - Robomaster 설치 : https://www.dji.com/kr/robomaster-s1/downloads
   - Robomaster SDK : https://robomaster-dev.readthedocs.io/en/latest/
2. 완성코드 사용 : 


## 주요 스크립트
- `Gotogoal.py`: 목표 지점까지 이동하는 기능.
- `Avoidobstacle.py`: 장애물 회피 기능.
- `Consensus.py`: 다중 에이전트 간 합의.
- `Formation.py`: 대형 형성 기능.
- `Leaderfollowing.py`: 리더 팔로잉 기능.
- `Cyclicpursuit.py`: 순환 추적 기능.


## 사용법 (Usage)

각 스크립트는 특정 제어 알고리즘을 테스트하고 시뮬레이션하는 데 사용됩니다. 예를 들어, 로봇이 목표 지점으로 이동하도록 하려면 아래 명령어를 실행하세요:

```bash
python Gotogoal.py
```

다른 알고리즘도 비슷한 방식으로 실행할 수 있습니다:

```bash
python Avoidobstacle.py
python Consensus.py
python Formation.py
python Leaderfollowing.py
python Cyclicpursuit.py
```

이 프로젝트는 충남대학교 모빌리티 실험실에서 수행한 것입니다.

자세한 영상을 보시려면 https://www.youtube.com/playlist?list=PLzxTrGXs-YYCN0y-ZgE6PvbbiA0Lm5HWJ 를 참고해주세요
