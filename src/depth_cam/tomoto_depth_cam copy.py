import pyrealsense2 as rs
import numpy as np
import cv2

# Realsense 카메라 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Realsense 카메라 시작
pipeline.start(config)

try:
    while True:
        # 프레임 가져오기
        frames = pipeline.wait_for_frames()

        # 컬러 프레임 가져오기
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        # 뎁스 프레임 가져오기
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())

        # 컬러 이미지 출력
        cv2.imshow("Color Image", color_image)

        # 뎁스 이미지 출력
        depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 뎁스 이미지 시각화를 위한 스케일 조정
        cv2.imshow("Depth Image", depth_image)

        # 종료 조건: q 키를 누르면 종료
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    # 리소스 해제
    pipeline.stop()
    cv2.destroyAllWindows()
