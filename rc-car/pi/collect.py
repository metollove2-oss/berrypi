"""데이터 수집 — data/run_XX/ 에 이미지와 labels.csv(filename,steering) 저장

TODO (팀원 1)
- 모드: auto(OpenCV 주행 + 자동 라벨) / manual(키보드·웹 조작) / recovery(가장자리→중앙 복귀만 녹화)
- 카메라 원본(BGR)을 jpg로 저장 — 전처리는 학습 때 적용하므로 저장 단계에서 하지 않음
- config.SAVE_HZ 주기로 저장, 녹화 시작/정지 키 지원
실행: python -m pi.collect --mode auto
"""
