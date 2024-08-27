# Robomaster EP 기반 군집 로봇 테스트베드 구축

<table>
  <tr>
    <td><img src="URL_OF_GIF_1" width="200"/></td>
    <td><img src="URL_OF_GIF_2" width="200"/></td>
    <td><img src="URL_OF_GIF_3" width="200"/></td>
  </tr>
  <tr>
    <td><img src="URL_OF_GIF_4" width="200"/></td>
    <td><img src="URL_OF_GIF_5" width="200"/></td>
    <td><img src="URL_OF_GIF_6" width="200"/></td>
  </tr>
  <tr>
    <td colspan="3" align="center"><img src="URL_OF_GIF_7" width="600"/></td>
  </tr>
</table>


## 설명
실내 측위 장비 두리시스템사의 Optitrack 장비를 기반으로 구축하였습니다.


## 설치 (Installation)

- HW
  - Optitrack 장비 사용

- SW
1. 개발환경
   - Robomaster 설치 : https://www.dji.com/kr/robomaster-s1/downloads
   - Robomaster SDK : https://robomaster-dev.readthedocs.io/en/latest/
2. 완성코드 사용 : 


## 주요 스크립트
- `Gotogoal.py`: 목표 지점까지 이동하는 알고리즘.
- `Avoidobstacle.py`: 장애물 회피 알고리즘.
- `Consensus.py`: 다중 에이전트 간 합의 알고리즘.
- `Formation.py`: 대형 형성 알고리즘.
- `Leaderfollowing.py`: 리더 팔로잉 알고리즘.
- `Cyclicpursuit.py`: 순환 추적 알고리즘.


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

