# 자율 주행 시스템 프로젝트

이 프로젝트는 ROS(로봇 운영 체제)를 기반으로 한 자율 주행 시스템으로, 카메라와 LiDAR 데이터를 활용하여 차선 감지, 장애물 회피 기능을 구현합니다. 아두이노를 통해 하드웨어 제어를 통합하며, 다양한 모듈이 협력하여 차량을 제어합니다.

---

## 프로젝트 일정
  ![image](https://github.com/user-attachments/assets/1991e81d-a522-46cd-b903-6b0ae9e89cfa)


## 프로젝트 수행
  ![image](https://github.com/user-attachments/assets/fa357a36-c20f-4816-aa7b-daa3b8d67345)
  https://www.youtube.com/watch?v=FwGlec1eLXw&t=5435
  
  https://www.youtube.com/watch?v=FwGlec1eLXw&t=6775
  
## 목차
- [시스템 개요](#시스템-개요)
- [제어 흐름](#제어-흐름)
- [모듈 다이어그램](#모듈-다이어그램)
- [모듈 설명](#모듈-설명)
  - [Obstacle Package](#obstacle-package)
  - [Lane Detection](#lane-detection)
  - [Obstacle Avoidance](#obstacle-avoidance)
  - [Horse Power Control](#horse-power-control)
  - [ROS Environment](#ros-environment)
---

## 시스템 개요
- **목표**: 카메라로 차선을 감지하고, LiDAR로 장애물을 탐지하여 차량을 주행시키는 시스템
- **구성 요소**:
  - **카메라**: 차선 감지
  - **LiDAR**: 장애물 감지 및 회피
  - **아두이노**: 모터 제어
  - **ROS**: 모듈 간 통신 및 데이터 처리
- **주요 기능**:
  - 차선 추적
  - 장애물 회피 (FSM 기반 상태 전이)

---

## 제어 흐름 다이어그램

```mermaid
graph TD
    A[사용자 명령<br>Stopline.py] --> B[ROS 초기화<br>Stopline.py: rospy]
    B --> C[센서 데이터 수집<br>horse_power_sensor.py: HPSensor]
    C --> D[HP.control 호출<br>Stopline.py -> horse_power.py: HP]
    D --> E[장애물 확인<br>horse_power.py: HP -> Obstacle_detector.py: Clustering]
    E -->|장애물 없음| F[차선 주행<br>horse_power.py: HP -> LaneDetector, Stanley]
    E -->|장애물 있음| G[장애물 회피<br>horse_power.py: HP -> Obstacle_detector.py: Clustering, FSM.py: FiniteStateMachine]
    F --> H[LaneDetector.process<br>lane_detector.py: LaneDetector]
    H --> I[Stanley.control<br>controller.py: Stanley]
    G --> J[시간 기반 회피 시퀀스<br>horse_power.py: HP]
    I --> K[속도/조향각 발행<br>horse_power.py: HP -> /ackermann_cmd]
    J --> K
    K --> L[Obstacle.ino<br>ROS 메시지 수신]
    L --> M[모터 제어<br>Obstacle.ino]
    M --> O[사용자 종료<br>Stopline.py]
    O --> P[시스템 종료<br>Stopline.py: cv2]

```
---

## 모듈 다이어그램

```mermaid
graph TD
    subgraph "Obstacle Package"
        Stopline[Stopline.py] -->|initializes| HP[horse_power.py - HP]
        Stopline -->|imports| Camera[camera.py - Camera]
        Stopline -->|imports| StoplineDetector[stopline_detector.py]
        ObstacleINO[Obstacle.ino] -->|subscribes /ackermann_cmd| HP
    end

    subgraph "Lane Detection"
        LaneDetector[LaneDetector] -->|processes| Camera
    end

    subgraph "Obstacle Avoidance"
        Clustering[Clustering] -->|uses| FSM[FSM.py - FiniteStateMachine]
    end

    subgraph "Horse Power Control"
        HP -->|uses| LaneDetector
        HP -->|uses| Clustering
        HP -->|uses| HPSensor[horse_power_sensor.py - HPSensor]
        HP -->|uses| Stanley[controller.py - Stanley]
    end

    subgraph "ROS Environment"
        Launch[Launch File] -->|launches| Stopline
        Launch -->|includes| LoCamera[lo_camera.launch]
        Stopline -->|initializes| ROSPY[rospy]
        HP -->|initializes| ROSPY
        Clustering -->|initializes| ROSPY
        HP -->|publishes /ackermann_cmd| Ackermann[AckermannDriveStamped]
        Clustering -->|publishes /ackermann_cmd| Ackermann
        ObstacleINO -->|subscribes /ackermann_cmd| Ackermann
        HPSensor -->|subscribes /camera0/usb_cam/image_raw| Image[sensor_msgs.msg.Image]
        HPSensor -->|sub BOTHscribes /scan_filtered| LaserScan[sensor_msgs.msg.LaserScan]
    end

    subgraph "External Modules"
        ROSPY[rospy]
        Ackermann[ackermann_msgs.msg]
        CV2[cv2]
        NP[numpy]
        SKLearn[sklearn.cluster.DBSCAN]
        TIME[time]
        MATH[math]
        CVBridge[cv_bridge.CvBridge]
        Image[sensor_msgs.msg.Image]
        LaserScan[sensor_msgs.msg.LaserScan]
        SUBPROCESS[subprocess]
    end

    subgraph "Arduino Environment"
        ObstacleINO -->|uses| CarLibrary[Car_Library.h]
    end

    Stopline -->|uses| CV2
    Stopline -->|uses| NP
    Stopline -->|uses| SUBPROCESS
    LaneDetector -->|uses| CV2
    LaneDetector -->|uses| NP
    Clustering -->|uses| NP
    Clustering -->|uses| SKLearn
    HP -->|uses| CV2
    HP -->|uses| TIME
    Camera -->|uses| CV2
    Camera -->|uses| NP
    FSM -->|uses| TIME
    HPSensor -->|uses| CVBridge
    HPSensor -->|uses| NP
    Stanley -->|uses| MATH
    Stanley -->|uses| NP
    StoplineDetector -->|uses| CV2
    StoplineDetector -->|uses| NP

```

## 모듈 설명
# Obstacle Package
- Stopline.py:
    - 역할: 메인 노드로 시스템 실행을 담당
    - 기능: HP를 호출하여 차량 제어 루프 실행
    - 의존성: horse_power.py, cv2
    - 
      ![image](https://github.com/user-attachments/assets/d0fd08d1-7969-4363-8656-de8b771cfc69)

- Obstacle.ino:
    - 역할: ROS 메시지를 받아 모터 제어
    - 기능: /ackermann_cmd 구독으로 속도/조향각 적용
    - 의존성: Car_Library.h
    - 
      ![image](https://github.com/user-attachments/assets/b5750ffe-2498-4bae-9f81-5ac20db5fbc7)


# Lane Detection
- camera.py:
    - 역할: 도로 검출을 위한 전처리 파이프라인을 구성
    - 기능: 노이즈 제거, 모폴로지 연산, 그레이스케일 변환, 블러링, 엣지 검출, 원근 변환, 반사광 제거
    - 의존성: numpy, cv2
    -
      ![image](https://github.com/user-attachments/assets/ab8b8527-3d58-404c-be9e-e4933727fcd7)

- LaneDetector:
    - 역할: 카메라 이미지에서 차선 감지 및 조향각 계산
    - 기능: Bird's Eye View 변환 후 차선 곡률 계산
    - 의존성: camera.py, cv2, numpy
    - 
      ![image](https://github.com/user-attachments/assets/d73961df-2957-47a8-b9d6-a837569d1ac4)



  
# Obstacle Avoidance
- Obstacle_detector.py - Clustering:
    - 역할: LiDAR 데이터로 장애물 클러스터링 및 회피 조향각 계산
    - 기능: DBSCAN으로 장애물 군집화, FSM으로 회피 방향 결정
    - 의존성: FSM.py, numpy, sklearn.cluster.DBSCAN
    - 
      ![image](https://github.com/user-attachments/assets/099b20dd-731b-4250-bd8b-1fa465a27af4)

- FSM.py - FiniteStateMachine:
    - 역할: 장애물 감지 횟수 기반 상태 전이
    - 기능: FollowLane, AvoidLeft, AvoidRight 상태 관리
    - 의존성: time
    - 
      ![image](https://github.com/user-attachments/assets/a0a50773-7c22-4f3f-87cd-b1630e56c633)
  
# Horse Power Control
- horse_power.py - HP:
    - 역할: 차량 제어 통합 (차선 주행 + 장애물 회피).
    - 기능:
        - Clustering으로 장애물 회피
        - 시간 기반 회피 시퀀스 실행
        - LaneDetector로 차선 추적
        - Stanley로 조향각 보정
    - 의존성: LaneDetector, Clustering, horse_power_sensor.py, controller.py, cv2, time
    - ROS: /ackermann_cmd 퍼블리시
    - 
      ![image](https://github.com/user-attachments/assets/7dcdf1ec-27cf-4ae1-82c1-6187f563c2d4)

  
- horse_power_sensor.py - HPSensor:
    - 역할: 센서 데이터 수집 (카메라, LiDAR)
    - 기능: ROS 토픽 구독으로 데이터 저장 (cam, lidar_filtered)
    - 의존성: cv_bridge, numpy
    - ROS: /camera0/usb_cam/image_raw, /scan_filtered 구독
    - 
      ![image](https://github.com/user-attachments/assets/65ff0fe2-c9d0-49d6-8055-b8d859efda52)


- controller.py - Stanley:
    - 역할: Stanley 제어 알고리즘으로 조향각 계산
    - 기능: 곡률 기반 조향각 반환
    - 의존성: math, numpy
    - 
      ![image](https://github.com/user-attachments/assets/c4b7ae4e-6c3c-421b-a3ef-d65ad62b71f7)


  
# ROS Environment
- 토픽:
    - /ackermann_cmd: HP와 Clustering에서 퍼블리시, Obstacle.ino에서 구독
    - /camera0/usb_cam/image_raw: HPSensor에서 카메라 이미지 수신
    - /scan_filtered: HPSensor에서 LiDAR 데이터 수신
- 런치 파일: Stopline.py 실행 및 카메라 설정(lo_camera.launch)
