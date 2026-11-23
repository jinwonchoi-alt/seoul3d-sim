# seoul3d-sim

서울 3D 지도의 교통 시뮬레이션(SUMO) 자동 갱신.

- 2시간마다 서울시 TOPIS 검지기 실측 교통량을 받아 25개 구 시뮬레이션을 다시 만든다.
- 결과는 GitHub Pages(`/sim/<구코드>.json`)로 올라가고, 3D 지도가 이 주소에서 읽는다.
- 도로망: 국가표준노드링크(MOCT_LINK) → `nets/<구코드>.net.xml.gz` (도로망이 바뀌면 PC의 sim.bat 으로 다시 만들어 교체)
- 수동 실행: Actions → seoul-sim → Run workflow
