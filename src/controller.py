#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from inputs import get_gamepad
from xycar_msgs.msg import xycar_motor
import rospy
import signal
import sys
import threading

# ROS 노드 초기화
rospy.init_node('Controller')
motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
motor_msg = xycar_motor()
speed = 0
angle = 0
lock = threading.Lock()

# 모터 제어 함수
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

# 정규화 함수
def normalize(mini, maxi, a, a_min=0, a_max=255):
    """
    주어진 값을 특정 범위로 선형 정규화하는 함수

    :param a: 정규화할 값
    :param a_min: 원래 범위의 최소값
    :param a_max: 원래 범위의 최대값
    :param mini: 새로운 범위의 최소값
    :param maxi: 새로운 범위의 최대값
    :return: 정규화된 값
    """
    a_normalized = mini + (a - a_min) * (maxi - mini) / (a_max - a_min)
    return a_normalized

# 종료 시그널 처리 함수
def signal_handler(sig, frame):
    print('Shutting down gracefully')
    sys.exit(0)

# 종료 시그널 연결
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# 입력 처리 함수 (비블로킹)
def input_thread():
    global speed, angle
    while not rospy.is_shutdown():
        events = get_gamepad()
        with lock:
            for event in events:
                if event.ev_type == "Key":
                    print(f"버튼 {event.code} 눌림/해제: {event.state}")
                elif event.ev_type == "Absolute":
                    if event.code == "ABS_Y":  # 왼쪽 패드 앞 위로
                        speed = normalize(50, -50, event.state)
                        speed = int(speed)
                    elif event.code == "ABS_Z":  # 오른쪽 패드 좌 우로
                        angle = normalize(-50, 50, event.state)
                        angle = int(angle)
                        print("event.state:", event.state)
                    print('speed :', speed, 'angle :', angle)
                
# 메인 제어 함수
def control():
    global speed, angle
    rate = rospy.Rate(10)  # 10Hz 주기로 발행
    while not rospy.is_shutdown():
        with lock:
            drive(angle, speed)  # 입력 이벤트가 없어도 주기적으로 발행
        rate.sleep()

if __name__ == '__main__':
    try:
        input_thread = threading.Thread(target=input_thread)
        input_thread.daemon = True
        input_thread.start()
        control()
    except rospy.ROSInterruptException:
        pass
