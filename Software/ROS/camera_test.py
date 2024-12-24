import cv2

# 최대 3개의 카메라 장치 확인 (더 많은 장치가 있을 경우 범위 조정 가능)
for i in range(3):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera {i} is active")
        cap.release()  # 카메라를 열고 나서 바로 닫음
    else:
        print(f"Camera {i} is not active")
