# 모빌리티 자율주행 프로젝트 (A407)

<!-- 필수 항목 -->

## 카테고리

| Application | Domain | Language | Framework |
| ---- | ---- | ---- | ---- |
| :black_square_button: Desktop Web | :white_check_mark: AI | :black_square_button: JavaScript | :black_square_button: Vue.js |
| :black_square_button: Mobile Web | :white_check_mark: Mobility | :black_square_button: TypeScript | :black_square_button: React |
| :black_square_button: Responsive Web | :black_square_button: Blockchain | :white_check_mark: C/C++ | :white_check_mark: ROS |
| :black_square_button: Android App | :black_square_button: IoT | :black_square_button: C# | :black_square_button: Node.js |
| :black_square_button: iOS App | :black_square_button: AR/VR/Metaverse | :white_check_mark: Python | :black_square_button: Flask/Django |
| :black_square_button: Desktop App | :black_square_button: Big Data | :black_square_button: Java | :black_square_button: Spring/Springboot |
| | | :black_square_button: Kotlin | |

<!-- 필수 항목 -->

## 프로젝트 소개

* 프로젝트명: 모빌리티 자율주행 프로젝트
* 서비스 특징: ROS 프레임워크를 이용해 시뮬레이터 Morai와의 통신을 통한 자율주행
* 주요 기능
  - GPS를 이용해 설정한 Global Path 경로 추종
  - Camera를 이용한 차선 인식 및 Object Detection
  - LiDAR를 이용한 3D Object Detection
* 주요 기술
  - ROS
  - MORAI
* 참조 리소스
  
* 배포 환경
  - URL: 
  - 테스트 계정: 

<!-- 자유 양식 -->
## 팀원 소개
* 김규리(팀장)
* 김승준
* 김준우
* 류태규
* 안창용
* 최명서

## 팀 Notion
- URL : https://www.notion.so/9d12d17211f246438662434b6a7ffa77

<!-- 자유 양식 -->
## 프로젝트 상세 설명
* 기능 상세 설명
  * GPS 좌표를 이용한 경로 추종
    * WGS84 3차원 좌표계를 UTM 2차원 좌표계로 변환
    * 차량 Localization 후 Pure Pursuit 알고리즘을 이용해 Global path 경로 추종
  * Camera를 이용한 Object Detection
    * Camera에서 받아온 영상 데이터를 OpenCV 모델을 이용해 각 Object Classification 및 Line detection
    * YOLO 모델을 이용한 표지판 및 신호등 인식
  * LiDAR를 이용한 3D Object Detection
    * YOLO를 이용한 3D Object Detection
