/*******************************************************
 * 장애물 회피 전용 아두이노 코드
 * ROS를 통해 조향각과 속도를 받아 모터를 제어하고 가변저항 값을 퍼블리시
*******************************************************/

#include <ros.h>  // ROS 통신 라이브러리
#include <ackermann_msgs/AckermannDriveStamped.h>  // Ackermann 메시지 타입
#include <Car_Library.h>  // 모터 제어 함수 라이브러리
#include <avr/io.h>  // AVR 입출력 레지스터 조작
#include <avr/interrupt.h>  // 인터럽트 처리
#include <std_msgs/Int16MultiArray.h>  // ROS 메시지 배열 타입

// 전역 변수: ROS에서 수신한 속도와 조향각
int sub_speed = 0;  // 수신된 속도
int sub_angle = 0;  // 수신된 조향각

/*--------------------------------------------------
                      ROS SETTING
---------------------------------------------------*/
std_msgs::Int16MultiArray str_msg;  // 가변저항 값과 상태를 퍼블리시할 메시지
ros::Publisher unochatter("uno", &str_msg);  // /uno 토픽 퍼블리셔
ros::NodeHandle nh;  // ROS 노드 핸들

// 콜백 함수: /ackermann_cmd 토픽에서 데이터 수신
void ackermannCallback(const ackermann_msgs::AckermannDriveStamped &ackermann);
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermannSubscriber("/ackermann_cmd", &ackermannCallback);

/*--------------------------------------------------
                      INIT
---------------------------------------------------*/
int motor_speed = 0;  // 모터 속도 저장 변수

// ROS 메시지 콜백 함수: 속도와 조향각 업데이트
void ackermannCallback(const ackermann_msgs::AckermannDriveStamped &ackermann) {
  sub_speed = ackermann.drive.speed;  // 속도 값 저장
  sub_angle = ackermann.drive.steering_angle;  // 조향각 값 저장
  motor_speed = sub_speed;  // 모터 속도 동기화
}

// 가변저항 관련 설정
int R_value[2] = {0, 0};  // 퍼블리시할 값 배열 (현재값, 목표값)
int R_min = 50;  // 가변저항 최소값
int R_max = 207;  // 가변저항 최대값
int R_mid = 142;  // 가변저항 중간값
int R_now = 0;  // 현재 가변저항 값
int R_pin = A4;  // 가변저항 아날로그 핀

// 차선 각도 범위 상수
const int MIN_LANE_ANGLE = -20;  // 최소 조향각
const int MAX_LANE_ANGLE = 20;   // 최대 조향각

// 제어 관련 변수
int REV = 0;  // 목표 가변저항 값 (Set Point)
double steering_output = 0.0;  // 조향 출력값
const double kp = 0.1;  // 비례 제어 상수 (P 제어)

// 모터 핀 설정
int motorF1 = 3;  // 조향 모터 핀 1
int motorF2 = 4;  // 조향 모터 핀 2
int motorB1 = 6;  // 구동 모터 핀 1 (왼쪽)
int motorB2 = 7;  // 구동 모터 핀 2 (왼쪽)

// 조향 제어 함수 (P 제어)
double control_steering_based_on_lane_angle(int target_resistance, int current_resistance) {
  double error = (target_resistance - current_resistance);  // 오차 계산: 목표값 - 현재값
  double steering_output = kp * error;  // 비례 제어: 출력 = kp * 오차
  return steering_output;
}

/*--------------------------------------------------
                      SETUP
---------------------------------------------------*/
void setup() {
  nh.getHardware()->setBaud(115200);  // ROS 통신 속도 설정
  nh.initNode();  // ROS 노드 초기화
  nh.advertise(unochatter);  // /uno 토픽 광고
  nh.subscribe(ackermannSubscriber);  // /ackermann_cmd 구독 설정

  // 타이머 설정: 1ms마다 인터럽트 발생
  TCCR2A = 2;  // CTC 모드 설정
  TCCR2B = 6;  // 프리스케일러 32
  TCNT2 = 0;  // 카운터 초기화
  OCR2A = 249;  // 1ms 주기 설정
  TIMSK2 = 2;  // 인터럽트 활성화

  // 핀 모드 설정
  pinMode(R_pin, INPUT);  // 가변저항 입력
  pinMode(motorF1, OUTPUT);  // 조향 모터 출력
  pinMode(motorF2, INPUT);  // 조향 모터 입력
  pinMode(motorB1, OUTPUT);  // 구동 모터 출력
  pinMode(motorB2, INPUT);  // 구동 모터 입력
  motor_hold(motorF1, motorF2);  // 조향 모터 초기 정지

  TCCR1B = TCCR1B & 0b11111000 | 1;  // PWM 주파수 31kHz로 설정 (소음 방지)
}

/*--------------------------------------------------
                      INTERRUPT
---------------------------------------------------*/
volatile int counter = 0;  // 인터럽트 카운터
const int read_interval = 2;  // 가변저항 읽기 간격 (2ms)

ISR(TIMER2_COMPA_vect) {
  counter++;  // 카운터 증가
  if (counter >= read_interval) {  // 2ms마다 실행
    R_now = analogRead(R_pin) / 4;  // 가변저항 값 읽기 (0~1023 -> 0~255)
    counter = 0;  // 카운터 초기화
  }
}

/*--------------------------------------------------
                      LOOP
---------------------------------------------------*/
void loop() {
  nh.spinOnce();  // ROS 메시지 처리

  publishing_Rvalue();  // 가변저항 값 퍼블리시

  // 구동 모터 제어: 속도와 조향각 기반
  motor_control_new(sub_angle, motor_speed);

  // 조향각에 따른 목표 가변저항 값 계산
  if (sub_angle > 0) {  // 우회전
    REV = map(sub_angle, 0, 20, R_mid, R_max);
  } else if (sub_angle == 0) {  // 직진
    REV = R_mid;
  } else {  // 좌회전
    REV = map(sub_angle, -20, 0, R_min, R_mid);
  }

  // 조향 출력 계산 및 모터 제어
  steering_output = control_steering_based_on_lane_angle(REV, R_now);
  float weight_multiple = 3.5;  // 출력 증폭 상수
  if (steering_output > 0) {  // 양수: 좌회전
    if (steering_output < 25) steering_output = 25;
    motor_backward(motorF1, motorF2, steering_output * weight_multiple);
  } else if (steering_output < 0) {  // 음수: 우회전
    if (steering_output > -25) steering_output = -25;
    motor_forward(motorF1, motorF2, abs(steering_output) * weight_multiple);
  } else {  // 0: 정지
    motor_hold(motorF1, motorF2);
  }
}

/*--------------------------------------------------
                      FUNCTION
---------------------------------------------------*/

// 구동 모터 제어 함수 (조향각과 속도 기반)
void motor_control_new(float lane_angle, int motor_speed) {
  int max_speed = 100;  // 최대 속도 설정
  float scaling_factor = 1.0;  // 속도 조정 스케일링
  float normalized_angle = abs(lane_angle) / 20.0;  // 조향각 정규화 (0~1)
  int speed = max_speed * (1.50 - (normalized_angle * scaling_factor));  // 중앙에서 멀어질수록 속도 감소

  if (motor_speed == 0) {  // 속도 0: 정지
    motor_hold(motorB1, motorB2);
  } else if (motor_speed > 0) {  // 전진
    if (speed == 50) {
      motor_forward(motorB1, motorB2, 60);  // 최소 속도 보정
    } else if (speed < 120) {
      motor_forward(motorB1, motorB2, 120);  // 기본 속도 설정
    } else {
      motor_forward(motorB1, motorB2, speed);
    }
  } else {  // 후진
    motor_backward(motorB1, motorB2, (abs(speed)) * 1.5);
  }
}

// 가변저항 값 퍼블리시 함수
void publishing_Rvalue() {
  R_value[0] = R_now;  // 현재 가변저항 값
  R_value[1] = REV;    // 목표 가변저항 값
  str_msg.data = R_value;  // 메시지 데이터 설정
  str_msg.data_length = 2;  // 데이터 길이
  unochatter.publish(&str_msg);  // /uno 토픽으로 퍼블리시
}
