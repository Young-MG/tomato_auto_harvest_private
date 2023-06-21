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
        ####################################################
        # 토마토 색상 검출및 중점 픽셀값 / 뎁스 좌표값
        ####################################################
        # hsv 이미지로 변환
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # 빨간색 영역의 범위를 정의
        lower_red = np.array([2, 50, 50])
        upper_red = np.array([9, 255, 255])
        # 빨간색 영역을 마스크
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        # 모폴로지 연산을 사용하여 마스크 개선
        kernel = np.ones((5,5),np.uint8)
        mask_red = cv2.erode(mask_red, kernel)
        mask_red = cv2.dilate(mask_red, kernel)
        # 빨간색 영역의 컨투어를 찾음
        contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 최소 컨투어 지정
        min_area = 2000
        # 가장큰 컨투어 추출
        if len(contours) > 0:
            i = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(i)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(i)
                cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                # 컨투어를 둘러싼 사각형의 중심점 추출
                M = cv2.moments(i)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                # 토마토 중점 픽셀값 출력
                print("pixel X :",cx, "pixel Y",cy)
                # 뎁스카메라 좌표 출력
                depth = depth_frame.get_distance(int(cx),int(cy))
                print("depth :",depth)
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                depth_pixel = [int(cx), int(cy)]
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth)
                print("depth point :",depth_point)
        ####################################################
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
