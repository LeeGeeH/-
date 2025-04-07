# 자율 주행 시스템 프로젝트

이 프로젝트는 ROS(로봇 운영 체제)를 기반으로 한 자율 주행 시스템으로, 카메라와 LiDAR 데이터를 활용하여 차선 감지, 정지선 감지, 장애물 회피 기능을 구현합니다. 자율 주행 차량의 핵심 기능을 통합적으로 테스트하고 개발할 수 있는 플랫폼을 제공합니다.

---

## 목차

- [주요 기능](#주요-기능)
- [프로젝트 구조](#프로젝트-구조)
- [ROS란 무엇인가](#ros란-무엇인가)
- [ROS 사용 이유](#ros-사용-이유)
- [프로젝트 장단점](#프로젝트-장단점)
- [모듈 설명](#모듈-설명)
- [경쟁 준비성](#경쟁-준비성)
- [설치 방법](#설치-방법)
- [사용 방법](#사용-방법)
- [디렉토리 구조](#디렉토리-구조)
- [기여 방법](#기여-방법)
- [라이선스](#라이선스)

---

## 주요 기능

- **차선 감지**: 카메라 영상에서 차선을 추적하여 주행 경로를 생성.
- **정지선 감지**: 카메라 영상을 통해 정지선을 인식하고 주행 제어에 반영.
- **장애물 회피**: LiDAR 데이터를 분석하여 장애물을 감지하고 회피 조향각을 계산.
- **차량 제어**: 속도와 조향각을 조절하여 자율 주행 수행.

---

## 프로젝트 구조

### 모듈 연결 다이어그램

```mermaid
graph TD
    subgraph "Obstacle Package"
        Stopline[Stopline.py] -->|imports| StoplineDetector[StoplineDetector]
        Stopline -->|imports| HP[HP]
        Stopline -->|imports| Camera[Camera]
        StoplineDetector -->|processes| Camera
        ObstacleINO[Obstacle.ino] -->|controls| HP[HP]
    end

    subgraph "Lane Detection"
        LaneDetector[LaneDetector] -->|processes| Camera
    end

    subgraph "Obstacle Avoidance"
        Clustering[Clustering] -->|uses| FSM[FiniteStateMachine]
    end

    subgraph "ROS Environment"
        Launch[Launch File] -->|launches| Stopline
        Launch -->|includes| LoCamera[lo_camera.launch]
        Stopline -->|initializes| ROSPY[rospy]
        Clustering -->|initializes| ROSPY
        Clustering -->|publishes /ackermann_cmd| Ackermann[AckermannDriveStamped]
        ObstacleINO -->|subscribes /ackermann_cmd| Ackermann
        ObstacleINO -->|publishes /uno| ROSPY
    end

    subgraph "External Modules"
        ROSPY[rospy]
        Ackermann[ackermann_msgs.msg]
        CV2[cv2]
        NP[numpy]
        SKLearn[sklearn.cluster.DBSCAN]
    end

    Stopline -->|uses| CV2
    Stopline -->|uses| NP
    StoplineDetector -->|uses| CV2
    StoplineDetector -->|uses| NP
    LaneDetector -->|uses| CV2
    LaneDetector -->|uses| NP
    Clustering -->|uses| NP
    Clustering -->|uses| SKLearn
